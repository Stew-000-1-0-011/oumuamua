/// @file robot_sim.cpp
/// oumuamua の cmd_vel から先のシミュレータ。
///
/// - `cmd_vel` (機体座標系) を受け、全方向移動ロボットの平面運動を積分する (plant.cpp)
/// - sotoba_ros の objects.cpp と同じフィールド形状に対して 2D LiDAR をレイキャストし、`/scan` を出す
/// - 真の姿勢を `~/truth_pose` と TF (`field -> base_link_truth`) に出す (推定との比較用)
/// - 車輪オドメトリの代わりに、機体速度の実測を `~/body_velocity` に出す (滑りの分だけずれ、雑音が乗る)
/// - `scan_duration` > 0 なら、1 スキャンの光線を時間をずらして撮る (回転式 LiDAR の歪み)
///
/// フィールドの寸法と初期姿勢のパラメータ名・意味は sotoba_node と同じにしてあるので、
/// sotoba_node と同じパラメータファイルを読ませれば食い違わない。
/// `start_*` は sotoba と同じく「LiDAR の」初期姿勢で、機体の初期姿勢は LiDAR の取付から逆算する。

#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <variant>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <sotoba/math/se3.hpp>
#include <sotoba/math/vec.hpp>

#include "oumuamua_sim/plant.hpp"
#include "sotoba_ros/objects.hpp"

namespace {
	namespace math = sotoba::math;
	using oumuamua_sim::Plant;
	using oumuamua_sim::PlantParams;
	using oumuamua_sim::Pose2;
	using oumuamua_sim::Twist2;
	using sotoba::math::UVec3;
	using sotoba::math::Vec3;
	using sotoba_ros::SE3;
	using sotoba_ros::Surface;

	auto yaw_rotation(const double yaw) -> SE3 {
		return math::rot(math::ypr(Vec3{0.f, 0.f, static_cast<float>(yaw)}));
	}

	/// x 軸まわりに 180 度 (LiDAR を上下逆さに付けたとき)。sotoba_ros の objects.cpp と同じ
	auto upside_down() -> SE3 {
		return math::rot(math::ypr(Vec3{std::numbers::pi_v<float>, 0.f, 0.f}));
	}

	auto yaw_to_quaternion(const double yaw) -> geometry_msgs::msg::Quaternion {
		geometry_msgs::msg::Quaternion q{};
		q.z = std::sin(0.5 * yaw);
		q.w = std::cos(0.5 * yaw);
		return q;
	}

	class RobotSim final : public rclcpp::Node {
	public:
		RobotSim() : rclcpp::Node("robot_sim") {
			// --- フィールド (sotoba_node と同じ名前・意味) ---
			const sotoba_ros::ObjectsConfig defaults{};
			sotoba_ros::ObjectsConfig field{};
			field.wall_height = static_cast<float>(
				this->declare_parameter<double>("wall_height", static_cast<double>(defaults.wall_height))
			);
			field.wall_thickness = static_cast<float>(this->declare_parameter<double>(
				"wall_thickness", static_cast<double>(defaults.wall_thickness)
			));
			field.include_notes = this->declare_parameter<bool>("include_notes", defaults.include_notes);
			// make_objects は「LiDAR 座標系での姿勢」を返すので、原点・高さ 0・正立の LiDAR を指定して
			// フィールド座標系での姿勢を得る
			field.start_x = 0.f;
			field.start_y = 0.f;
			field.start_yaw = 0.f;
			field.lidar_height = 0.f;
			field.lidar_upside_down = false;
			for (const auto& object : sotoba_ros::make_objects(field)) {
				for (const auto& surface : object.surfaces) {
					// 物体の姿勢 (フィールド座標系) を曲面に焼き込んでおく
					Surface s = surface;
					std::visit([&](auto& v) { v.apply_se3(object.initial_pose); }, s);
					this->surfaces_.push_back(s);
				}
			}

			// --- LiDAR の取付 (base_link -> laser) ---
			const double lidar_height =
				this->declare_parameter<double>("lidar_height", static_cast<double>(defaults.lidar_height));
			const bool lidar_upside_down =
				this->declare_parameter<bool>("lidar_upside_down", defaults.lidar_upside_down);
			const double lidar_x = this->declare_parameter<double>("lidar_x", 0.0);
			const double lidar_y = this->declare_parameter<double>("lidar_y", 0.0);
			const double lidar_yaw = this->declare_parameter<double>("lidar_yaw", 0.0);
			this->base_to_lidar_ =
				math::trans(Vec3{static_cast<float>(lidar_x), static_cast<float>(lidar_y), static_cast<float>(lidar_height)})
				* yaw_rotation(lidar_yaw);
			if (lidar_upside_down) {
				this->base_to_lidar_ = this->base_to_lidar_ * upside_down();
			}

			// --- 初期姿勢 ---
			// start_* は LiDAR の姿勢 (sotoba と同じ)。機体の姿勢へ直してから、真値のずれを足す
			const double start_x = this->declare_parameter<double>("start_x", static_cast<double>(defaults.start_x));
			const double start_y = this->declare_parameter<double>("start_y", static_cast<double>(defaults.start_y));
			const double start_yaw =
				this->declare_parameter<double>("start_yaw", static_cast<double>(defaults.start_yaw));
			const double base_yaw = start_yaw - lidar_yaw;
			this->initial_ = Pose2{
				.x = start_x - (std::cos(base_yaw) * lidar_x - std::sin(base_yaw) * lidar_y)
					+ this->declare_parameter<double>("start_offset_x", 0.02),
				.y = start_y - (std::sin(base_yaw) * lidar_x + std::cos(base_yaw) * lidar_y)
					+ this->declare_parameter<double>("start_offset_y", -0.01),
				.yaw = base_yaw + this->declare_parameter<double>("start_offset_yaw", 0.01),
			};

			// --- 運動 ---
			const double rate = this->declare_parameter<double>("sim_rate", 200.0);
			this->declare_parameter<double>("cmd_timeout", 0.2);
			this->declare_parameter<double>("plant.tau_linear", 0.1);
			this->declare_parameter<double>("plant.tau_angular", 0.1);
			this->declare_parameter<double>("plant.max_accel_linear", 0.0);
			this->declare_parameter<double>("plant.max_accel_angular", 0.0);
			this->declare_parameter<double>("plant.velocity_scale_linear", 1.0);
			this->declare_parameter<double>("plant.velocity_scale_angular", 1.0);
			this->plant_.emplace(this->read_plant_params(), this->initial_);

			// --- スキャン ---
			this->frame_id_ = this->declare_parameter<std::string>("frame_id", "laser");
			this->ray_num_ = static_cast<std::size_t>(
				std::max<std::int64_t>(2, this->declare_parameter<std::int64_t>("ray_num", 720))
			);
			this->angle_min_ = static_cast<float>(this->declare_parameter<double>("angle_min", -std::numbers::pi));
			this->angle_max_ = static_cast<float>(this->declare_parameter<double>("angle_max", std::numbers::pi));
			this->range_min_ = static_cast<float>(this->declare_parameter<double>("range_min", 0.05));
			this->range_max_ = static_cast<float>(this->declare_parameter<double>("range_max", 30.0));
			this->declare_parameter<double>("range_noise_stddev", 0.005);
			this->declare_parameter<double>("scan_latency", 0.0);
			this->declare_parameter<double>("scan_duration", 0.0);
			const double scan_hz = this->declare_parameter<double>("scan_hz", 20.0);

			// --- 真値 ---
			this->field_frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->truth_frame_ = this->declare_parameter<std::string>("truth_frame", "base_link_truth");

			this->read_runtime_params();
			this->param_cb_ = this->add_post_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter>&) { this->read_runtime_params(); }
			);

			const auto cmd_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
			const bool cmd_vel_stamped = this->declare_parameter<bool>("cmd_vel_stamped", false);
			const double body_velocity_rate = this->declare_parameter<double>("body_velocity_rate", 50.0);
			this->declare_parameter<double>("body_velocity_noise_linear", 0.02);
			this->declare_parameter<double>("body_velocity_noise_angular", 0.05);
			this->read_runtime_params();
			const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
			if (cmd_vel_stamped) {
				this->cmd_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
					cmd_topic, 10, [this](const geometry_msgs::msg::TwistStamped& m) { this->on_cmd(m.twist); }
				);
			} else {
				this->cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
					cmd_topic, 10, [this](const geometry_msgs::msg::Twist& m) { this->on_cmd(m); }
				);
			}
			this->body_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/body_velocity", 10);
			this->body_velocity_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(1.0 / body_velocity_rate), [this] { this->publish_body_velocity(); }
			);
			this->scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS{});
			this->truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/truth_pose", 10);
			this->truth_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/truth_odom", 10);
			this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
			this->reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reset",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					this->plant_->reset(this->initial_);
					this->history_.clear();
					res->success = true;
				}
			);

			this->dt_ = 1.0 / rate;
			this->sim_timer_ = this->create_wall_timer(std::chrono::duration<double>(this->dt_), [this] { this->step(); });
			this->scan_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(1.0 / scan_hz), [this] { this->publish_scan(); }
			);

			RCLCPP_INFO(
				this->get_logger(),
				"%zu surfaces, start (%.3f, %.3f, %.3f), lidar %s",
				this->surfaces_.size(),
				this->initial_.x,
				this->initial_.y,
				this->initial_.yaw,
				lidar_upside_down ? "upside down" : "upright"
			);
		}

	private:
		void on_cmd(const geometry_msgs::msg::Twist& m) {
			this->cmd_ = Twist2{.vx = m.linear.x, .vy = m.linear.y, .omega = m.angular.z};
			this->cmd_received_ = this->now();
		}

		/// 車輪オドメトリの模擬。車輪は地面との滑りを知らないので、実際の速度を velocity_scale で割り戻す
		void publish_body_velocity() {
			const auto& v = this->plant_->velocity();
			const auto params = this->read_plant_params();
			const auto noise = [this](const double sigma) {
				return sigma > 0.0 ? std::normal_distribution<double>{0.0, sigma}(this->rng_) : 0.0;
			};
			geometry_msgs::msg::TwistStamped m{};
			m.header.stamp = this->now();
			m.header.frame_id = "base_link";
			m.twist.linear.x = v.vx / params.velocity_scale_linear + noise(this->body_noise_linear_);
			m.twist.linear.y = v.vy / params.velocity_scale_linear + noise(this->body_noise_linear_);
			m.twist.angular.z = v.omega / params.velocity_scale_angular + noise(this->body_noise_angular_);
			this->body_velocity_pub_->publish(m);
		}

		auto read_plant_params() -> PlantParams {
			return PlantParams{
				.tau_linear = this->get_parameter("plant.tau_linear").as_double(),
				.tau_angular = this->get_parameter("plant.tau_angular").as_double(),
				.max_accel_linear = this->get_parameter("plant.max_accel_linear").as_double(),
				.max_accel_angular = this->get_parameter("plant.max_accel_angular").as_double(),
				.velocity_scale_linear = this->get_parameter("plant.velocity_scale_linear").as_double(),
				.velocity_scale_angular = this->get_parameter("plant.velocity_scale_angular").as_double(),
			};
		}

		void read_runtime_params() {
			this->cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
			this->range_noise_ = static_cast<float>(this->get_parameter("range_noise_stddev").as_double());
			this->scan_latency_ = this->get_parameter("scan_latency").as_double();
			this->scan_duration_ = std::max(0.0, this->get_parameter("scan_duration").as_double());
			if (this->has_parameter("body_velocity_noise_angular")) {
				this->body_noise_linear_ = this->get_parameter("body_velocity_noise_linear").as_double();
				this->body_noise_angular_ = this->get_parameter("body_velocity_noise_angular").as_double();
			}
			if (this->plant_) {
				this->plant_->set_params(this->read_plant_params());
			}
		}

		void step() {
			const auto now = this->now();
			Twist2 cmd{};
			if (this->cmd_received_ && (now - *this->cmd_received_).seconds() <= this->cmd_timeout_) {
				cmd = this->cmd_;
			}
			this->plant_->step(cmd, this->dt_);
			const auto& pose = this->plant_->pose();

			// スキャンの遅れ用に、少しだけ姿勢の履歴を持つ
			this->history_.push_back({now, pose});
			while (this->history_.size() > 2
				&& (now - this->history_.front().first).seconds() > this->scan_latency_ + this->scan_duration_ + 0.5) {
				this->history_.pop_front();
			}

			this->publish_truth(now, pose);
		}

		/// 時刻 t の真の姿勢 (履歴から最も近いもの)
		auto pose_at(const rclcpp::Time& t) const -> Pose2 {
			for (auto it = this->history_.rbegin(); it != this->history_.rend(); ++it) {
				if (it->first <= t) {
					return it->second;
				}
			}
			return this->history_.empty() ? this->plant_->pose() : this->history_.front().second;
		}

		/// 機体の姿勢 -> フィールドを LiDAR 座標系へ写す変換
		auto lidar_from_field(const Pose2& base) const -> SE3 {
			const SE3 field_to_base =
				math::trans(Vec3{static_cast<float>(base.x), static_cast<float>(base.y), 0.f}) * yaw_rotation(base.yaw);
			return (field_to_base * this->base_to_lidar_).inverse();
		}

		void publish_scan() {
			// scan_latency だけ前に撮り終えたスキャンが、今届いたことにする。
			// 時刻は撮り始め (LaserScan の約束)。光線 i は撮り始めから i * time_increment 後に撮る
			const auto now = this->now();
			const auto end = now - rclcpp::Duration::from_seconds(this->scan_latency_);
			const auto stamp = end - rclcpp::Duration::from_seconds(this->scan_duration_);

			sensor_msgs::msg::LaserScan scan{};
			scan.header.stamp = stamp;
			scan.header.frame_id = this->frame_id_;
			scan.angle_min = this->angle_min_;
			scan.angle_max = this->angle_max_;
			scan.angle_increment = (this->angle_max_ - this->angle_min_) / static_cast<float>(this->ray_num_);
			scan.time_increment = static_cast<float>(this->scan_duration_ / static_cast<double>(this->ray_num_));
			scan.scan_time = static_cast<float>(this->scan_duration_);
			scan.range_min = this->range_min_;
			scan.range_max = this->range_max_;
			scan.ranges.reserve(this->ray_num_);

			// 歪みが無ければ全光線で同じ変換なので、曲面を一度だけ動かす
			std::vector<Surface> surfaces = this->surfaces_;
			const bool distorted = this->scan_duration_ > 0.0;
			if (!distorted) {
				const SE3 T = this->lidar_from_field(this->pose_at(stamp));
				for (auto& s : surfaces) {
					std::visit([&](auto& v) { v.apply_se3(T); }, s);
				}
			}

			for (std::size_t i = 0; i < this->ray_num_; ++i) {
				if (distorted) {
					const auto t = stamp + rclcpp::Duration::from_seconds(scan.time_increment * static_cast<double>(i));
					const SE3 T = this->lidar_from_field(this->pose_at(t));
					for (std::size_t k = 0; k < surfaces.size(); ++k) {
						surfaces[k] = this->surfaces_[k];
						std::visit([&](auto& v) { v.apply_se3(T); }, surfaces[k]);
					}
				}
				const float angle = scan.angle_min + scan.angle_increment * static_cast<float>(i);
				const UVec3 ray{std::cos(angle), std::sin(angle), 0.f};
				float nearest2 = std::numeric_limits<float>::infinity();
				for (const auto& s : surfaces) {
					std::visit([&](const auto& v) { nearest2 = std::min(nearest2, v.ray_collision(ray)); }, s);
				}
				float d = std::sqrt(nearest2);
				if (!std::isfinite(d) || d > this->range_max_) {
					scan.ranges.push_back(std::numeric_limits<float>::infinity());
					continue;
				}
				if (this->range_noise_ > 0.f) {
					d += std::normal_distribution<float>{0.f, this->range_noise_}(this->rng_);
				}
				scan.ranges.push_back(d);
			}
			this->scan_pub_->publish(scan);
		}

		void publish_truth(const rclcpp::Time& now, const Pose2& pose) {
			geometry_msgs::msg::PoseStamped msg{};
			msg.header.stamp = now;
			msg.header.frame_id = this->field_frame_;
			msg.pose.position.x = pose.x;
			msg.pose.position.y = pose.y;
			msg.pose.orientation = yaw_to_quaternion(pose.yaw);
			this->truth_pub_->publish(msg);

			nav_msgs::msg::Odometry odom{};
			odom.header = msg.header;
			odom.child_frame_id = this->truth_frame_;
			odom.pose.pose = msg.pose;
			const auto& v = this->plant_->velocity();
			odom.twist.twist.linear.x = v.vx;
			odom.twist.twist.linear.y = v.vy;
			odom.twist.twist.angular.z = v.omega;
			this->truth_odom_pub_->publish(odom);

			geometry_msgs::msg::TransformStamped tf{};
			tf.header = msg.header;
			tf.child_frame_id = this->truth_frame_;
			tf.transform.translation.x = pose.x;
			tf.transform.translation.y = pose.y;
			tf.transform.rotation = msg.pose.orientation;
			this->tf_broadcaster_->sendTransform(tf);
		}

		std::vector<Surface> surfaces_{};
		SE3 base_to_lidar_{};
		Pose2 initial_{};
		std::optional<Plant> plant_{};
		double dt_{};
		double cmd_timeout_{};
		Twist2 cmd_{};
		std::optional<rclcpp::Time> cmd_received_{};
		std::deque<std::pair<rclcpp::Time, Pose2>> history_{};

		std::string frame_id_{};
		std::size_t ray_num_{};
		float angle_min_{};
		float angle_max_{};
		float range_min_{};
		float range_max_{};
		float range_noise_{};
		double scan_latency_{};
		double scan_duration_{};
		double body_noise_linear_{};
		double body_noise_angular_{};
		std::mt19937 rng_{0};

		std::string field_frame_{};
		std::string truth_frame_{};

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_{};
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_{};
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr body_velocity_pub_{};
		rclcpp::TimerBase::SharedPtr body_velocity_timer_{};
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_{};
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_{};
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_odom_pub_{};
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_{};
		rclcpp::TimerBase::SharedPtr sim_timer_{};
		rclcpp::TimerBase::SharedPtr scan_timer_{};
		rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotSim>());
	rclcpp::shutdown();
	return 0;
}
