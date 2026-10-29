// 吸引・位置制御を持たない昇降機構。速度 PI 制御は USB-CAN ボードが行う。
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "harite/velocity_command.hpp"

namespace {
class HariteNode final : public rclcpp::Node {
public:
	HariteNode() : Node("harite") {
		// 機体の値は oumuamua_bringup/config/harite.yaml に集約する。
		const auto motor = declare_parameter<std::int64_t>("motor");
		const auto temperature = declare_parameter<std::int64_t>("temperature_limit");
		const auto repeats = declare_parameter<std::int64_t>("config_repeat");
		const double rate = declare_parameter<double>("control_rate");
		const double kp = declare_parameter<double>("vel_kp");
		const double ki = declare_parameter<double>("vel_ki");
		if (motor < 1 || motor > 8 || temperature < 0 || temperature > 255 || repeats < 1) {
			throw std::invalid_argument("motor must be 1..8, temperature_limit 0..255, config_repeat >= 1");
		}
		if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0) {
			throw std::invalid_argument("control_rate must be finite and in (0, 1000] Hz");
		}
		for (const auto gain : {kp, ki}) {
			if (!std::isfinite(gain) || gain < 0.0 || gain > std::numeric_limits<float>::max()) {
				throw std::invalid_argument("velocity gains must be finite, nonnegative float32 values");
			}
		}
		config_.motor = static_cast<std::uint8_t>(motor - 1); // RobomasFrame は 0 始まり
		config_.c620 = declare_parameter<bool>("c620");
		config_.temp = static_cast<std::uint8_t>(temperature);
		config_.velkp = static_cast<float>(kp);
		config_.velki = static_cast<float>(ki);
		config_repeat_ = repeats;
		const double max_velocity = declare_parameter<double>("max_velocity");
		const double timeout = declare_parameter<double>("command_timeout");
		command_ = std::make_unique<harite::VelocityCommand>(max_velocity, timeout);
		const bool enabled = declare_parameter<bool>("enable_on_start");
		const auto frame_topic = declare_parameter<std::string>("robomas_frame_topic");
		auto target_topic = declare_parameter<std::string>("robomas_target_topic");
		if (target_topic.empty()) target_topic = "/robomas_target" + std::to_string(motor);
		frame_pub_ = create_publisher<robomas_plugins::msg::RobomasFrame>(frame_topic, rclcpp::QoS{10});
		target_pub_ = create_publisher<robomas_plugins::msg::RobomasTarget>(target_topic, rclcpp::QoS{1});
		command_pub_ = create_publisher<std_msgs::msg::Float64>("~/command", rclcpp::QoS{1});
		velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
			"~/target_velocity", rclcpp::QoS{1}, [this](const std_msgs::msg::Float64& msg) {
				const std::scoped_lock lock{mutex_};
				command_->receive(msg.data, harite::VelocityCommand::Clock::now());
				publish_target(); // 無効値でも直ちに 0 を送る
			});
		enable_srv_ = create_service<std_srvs::srv::SetBool>(
			"~/enable", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
				std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
				set_enabled(req->data);
				res->success = true;
				res->message = req->data ? "harite enabled (velocity control)" : "harite disabled";
			});
		set_enabled(enabled);
		timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {
			const std::scoped_lock lock{mutex_};
			publish_target();
		});
		config_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] {
			const std::scoped_lock lock{mutex_};
			if (config_sent_ < config_repeat_) send_config();
		});
		RCLCPP_INFO(get_logger(), "motor %ld (%s), velocity limit %.1f rotor rad/s -> %s",
			static_cast<long>(motor), config_.c620 ? "C620" : "C610", max_velocity, target_topic.c_str());
	}

	void stop() { set_enabled(false); }

private:
	void set_enabled(bool enabled) {
		const std::scoped_lock lock{mutex_};
		const bool changed = command_->enabled() != enabled;
		command_->enable(enabled);
		publish_target();
		// 同じ enable の繰り返しでボードの PI をリセットしない。
		if (changed || config_sent_ == 0) {
			config_sent_ = 0;
			send_config();
		}
	}
	void send_config() {
		config_.mode = command_->enabled() ? 1 : 0; // robomas_plugins: 1 = 速度、0 = 無効
		frame_pub_->publish(config_);
		++config_sent_;
	}
	void publish_target() {
		const double velocity = command_->target(harite::VelocityCommand::Clock::now());
		robomas_plugins::msg::RobomasTarget msg{};
		msg.target = static_cast<float>(velocity);
		target_pub_->publish(msg);
		std_msgs::msg::Float64 state{};
		state.data = velocity;
		command_pub_->publish(state); // 実測ではなくボードへ送った指令
	}

	std::mutex mutex_; // shutdown callback は executor と別のスレッドでも呼ばれる
	std::unique_ptr<harite::VelocityCommand> command_;
	robomas_plugins::msg::RobomasFrame config_{};
	std::int64_t config_repeat_{};
	std::int64_t config_sent_{};
	rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr frame_pub_;
	rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr target_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr config_timer_;
};
} // namespace

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<HariteNode> node;
	try {
		node = std::make_shared<HariteNode>();
		// shutdown 前に停止を送る。通信断や強制終了の場合の到達は保証できない。
		node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak = std::weak_ptr<HariteNode>(node)] {
			if (auto live = weak.lock()) live->stop();
		});
		rclcpp::spin(node);
	} catch (const std::exception& e) {
		RCLCPP_FATAL(rclcpp::get_logger("harite"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
