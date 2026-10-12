/// @file path_follower_node.cpp
/// 起動前に作った経路ファイルを読み、速度計画を付けて、軌道追従ノードに時刻ごとの目標を出す。
///
/// - 経路ファイル (CSV: x, y, yaw) を起動時に読み、`limits.*` を守る速度計画を作る。
///   経路は複数持てる (`path_dir` の下の `paths` の各名前 + `.csv`、または `path_file` 1 つ)
/// - `~/start/<経路の名前>` でその経路の始点から走り出し、`rate` の周期で目標 (位置・向き・速度FF) を
///   `reference_topic` に出す。`~/start` は最初の経路
/// - 機体が目標から `max_tracking_error` より離れたら、追いつくまで目標を進めない
/// - 終点に着いたら、終点を出し続けてそこに留まる (`hold_at_goal`)
/// - `~/stop` で目標を出すのをやめる (追従ノードは目標が途切れると止まる)

#include <chrono>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <holonomic_tracker/msg/tracking_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "path_follower/trajectory.hpp"

namespace {
	using holonomic_tracker::msg::TrackingReference;
	using path_follower::Limits;
	using path_follower::State;
	using path_follower::Trajectory;

	class PathFollowerNode final : public rclcpp::Node {
	public:
		PathFollowerNode() : rclcpp::Node{"path_follower"} {
			const auto path_file = this->declare_parameter<std::string>("path_file", "");
			const auto path_dir = this->declare_parameter<std::string>("path_dir", "");
			const auto path_names = this->declare_parameter<std::vector<std::string>>("paths", std::vector<std::string>{});
			const auto reference_topic = this->declare_parameter<std::string>("reference_topic", "/tracker_node/reference");
			const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/state_estimator/odom");
			this->frame_id_ = this->declare_parameter<std::string>("frame_id", "field");
			const double rate = this->declare_parameter<double>("rate", 50.0);
			if (!(rate > 0.0)) { throw std::invalid_argument("rate must be positive"); }

			const Limits limits{
				.max_speed = this->declare_parameter<double>("limits.max_speed", 0.5),
				.max_accel = this->declare_parameter<double>("limits.max_accel", 1.0),
				.max_lateral_accel = this->declare_parameter<double>("limits.max_lateral_accel", 1.0),
				.max_yaw_rate = this->declare_parameter<double>("limits.max_yaw_rate", 1.5),
			};
			this->start_tolerance_ = this->declare_parameter<double>("start_tolerance", 0.15);
			this->max_tracking_error_ = this->declare_parameter<double>("max_tracking_error", 0.15);
			this->odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
			this->hold_at_goal_ = this->declare_parameter<bool>("hold_at_goal", true);
			const bool auto_start = this->declare_parameter<bool>("auto_start", false);

			// 経路: path_dir/<名前>.csv を paths の順に。path_file があれば、それも (名前はファイル名から)
			std::vector<std::pair<std::string, std::string>> files{};
			if (!path_file.empty()) { files.emplace_back(std::filesystem::path{path_file}.stem().string(), path_file); }
			for (const auto& name : path_names) {
				if (name.empty()) { continue; }
				files.emplace_back(name, (std::filesystem::path{path_dir} / (name + ".csv")).string());
			}
			if (files.empty()) { throw std::invalid_argument("no path: set 'paths' (with 'path_dir') or 'path_file'"); }
			for (const auto& [name, file] : files) {
				std::ifstream in{file};
				if (!in) { throw std::invalid_argument("cannot open path file '" + file + "'"); }
				const auto& route = this->routes_.emplace_back(name, Trajectory{path_follower::load_path_csv(in), limits});
				const auto& first = route.trajectory.points().front();
				const auto& last = route.trajectory.points().back();
				RCLCPP_INFO(
					this->get_logger(), "path '%s' (%s): %zu points, %.2f m, %.1f s, from (%.3f, %.3f) to (%.3f, %.3f)",
					name.c_str(), file.c_str(), route.trajectory.points().size(), route.trajectory.length(),
					route.trajectory.duration(), first.x, first.y, last.x, last.y
				);
			}

			this->reference_pub_ = this->create_publisher<TrackingReference>(reference_topic, rclcpp::QoS{10});
			// RViz で見る用。後から起動しても届くよう transient local
			this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS{1}.transient_local());
			this->publish_path();

			this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				odom_topic, rclcpp::QoS{10},
				[this](const nav_msgs::msg::Odometry::ConstSharedPtr m) {
					this->odom_ = Pose{m->pose.pose.position.x, m->pose.pose.position.y, this->now()};
				}
			);
			// ~/start は最初の経路、~/start/<名前> はその経路
			const auto add_start = [this](const std::string& service, const std::size_t index) {
				this->start_srvs_.push_back(this->create_service<std_srvs::srv::Trigger>(
					service,
					[this, index](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
						res->success = this->start(index, res->message);
					}
				));
			};
			add_start("~/start", 0);
			for (std::size_t i = 0; i < this->routes_.size(); ++i) { add_start("~/start/" + this->routes_[i].name, i); }
			this->stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/stop",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					this->running_ = false;
					res->success = true;
					res->message = "stopped";
					RCLCPP_INFO(this->get_logger(), "stopped");
				}
			);

			this->period_ = 1.0 / rate;
			this->timer_ = this->create_wall_timer(std::chrono::duration<double>(this->period_), [this] { this->on_timer(); });
			if (auto_start) {
				this->running_ = true;
				RCLCPP_INFO(this->get_logger(), "auto_start: following path '%s' from its first point", this->routes_.front().name.c_str());
			}
		}

	private:
		struct Route {
			std::string name;
			Trajectory trajectory;
		};
		struct Pose {
			double x{};
			double y{};
			rclcpp::Time received{};
		};

		/// 新しい自己位置があればそれを返す
		auto fresh_odom() -> std::optional<Pose> {
			if (!this->odom_ || (this->now() - this->odom_->received).seconds() > this->odom_timeout_) { return std::nullopt; }
			return this->odom_;
		}

		auto active() const -> const Trajectory& { return this->routes_[this->active_].trajectory; }

		auto start(const std::size_t index, std::string& message) -> bool {
			const auto& route = this->routes_[index];
			// 始点から離れたところで始めると、追従ノードが始点へ一気に寄せようとする
			if (this->start_tolerance_ > 0.0) {
				const auto odom = this->fresh_odom();
				if (!odom) {
					message = "no recent pose estimate; not starting";
					RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
					return false;
				}
				const auto& first = route.trajectory.points().front();
				const double distance = std::hypot(odom->x - first.x, odom->y - first.y);
				if (distance > this->start_tolerance_) {
					message = "the robot is " + std::to_string(distance) + " m from the first point of path '" + route.name + "' (start_tolerance "
						+ std::to_string(this->start_tolerance_) + " m); not starting";
					RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
					return false;
				}
			}
			this->active_ = index;
			this->elapsed_ = 0.0;
			this->running_ = true;
			this->reported_goal_ = false;
			this->publish_path();
			message = "started '" + route.name + "'";
			RCLCPP_INFO(this->get_logger(), "started '%s' (%.1f s to the goal)", route.name.c_str(), route.trajectory.duration());
			return true;
		}

		void on_timer() {
			if (!this->running_) { return; }
			State state = this->active().sample(this->elapsed_);

			// 機体が遅れているときは目標を進めない (その場の目標を、速度FFなしで出す)
			bool waiting = false;
			if (this->max_tracking_error_ > 0.0 && !state.finished) {
				const auto odom = this->fresh_odom();
				if (!odom) {
					waiting = true;
					RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "no recent pose estimate; holding the reference");
				} else if (std::hypot(odom->x - state.x, odom->y - state.y) > this->max_tracking_error_) {
					waiting = true;
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 2000,
						"the robot is %.2f m behind the reference; waiting for it", std::hypot(odom->x - state.x, odom->y - state.y)
					);
				}
			}
			if (waiting) {
				state.vx = state.vy = state.omega = 0.0;
			} else {
				this->elapsed_ += this->period_;
			}

			if (state.finished && !this->reported_goal_) {
				this->reported_goal_ = true;
				RCLCPP_INFO(this->get_logger(), "reached the end of the path%s", this->hold_at_goal_ ? "; holding the goal" : "");
				if (!this->hold_at_goal_) { this->running_ = false; }
			}

			TrackingReference m{};
			m.header.stamp = this->now();
			m.header.frame_id = this->frame_id_;
			m.x = state.x;
			m.y = state.y;
			m.yaw = state.yaw;
			m.vx = state.vx;
			m.vy = state.vy;
			m.omega = state.omega;
			this->reference_pub_->publish(m);
		}

		void publish_path() {
			nav_msgs::msg::Path path{};
			path.header.stamp = this->now();
			path.header.frame_id = this->frame_id_;
			for (const auto& p : this->active().points()) {
				geometry_msgs::msg::PoseStamped pose{};
				pose.header = path.header;
				pose.pose.position.x = p.x;
				pose.pose.position.y = p.y;
				pose.pose.orientation.z = std::sin(0.5 * p.yaw);
				pose.pose.orientation.w = std::cos(0.5 * p.yaw);
				path.poses.push_back(pose);
			}
			this->path_pub_->publish(path);
		}

		std::vector<Route> routes_{};
		/// いま走っている (最後に走った) 経路
		std::size_t active_{0};
		std::string frame_id_{};
		double period_{};
		double start_tolerance_{};
		double max_tracking_error_{};
		double odom_timeout_{};
		bool hold_at_goal_{true};

		bool running_{false};
		bool reported_goal_{false};
		/// 走り始めてからの時間 [s]。機体を待っている間は進めない
		double elapsed_{0.0};
		std::optional<Pose> odom_{};

		rclcpp::Publisher<TrackingReference>::SharedPtr reference_pub_{};
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_{};
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
		std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> start_srvs_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
}

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<PathFollowerNode>());
	} catch (const std::invalid_argument& e) {
		RCLCPP_FATAL(rclcpp::get_logger("path_follower"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
