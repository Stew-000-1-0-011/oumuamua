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
/// - アクション `~/follow` (path_follower/action/FollowPath) は、名前付きの経路か、今の位置から経由点を直線でつないだ経路を走り、
///   終点で止まったら (自己位置が `goal_tolerance` 以内に `goal_settle_time` 留まったら) 成功を返す

#include <chrono>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <holonomic_tracker/msg/tracking_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "path_follower/action/follow_path.hpp"
#include "path_follower/trajectory.hpp"

namespace {
	using holonomic_tracker::msg::TrackingReference;
	using path_follower::Limits;
	using path_follower::State;
	using path_follower::Trajectory;
	using FollowPath = path_follower::action::FollowPath;
	using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

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

			const Limits limits = this->limits_ = Limits{
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
			this->spacing_ = this->declare_parameter<double>("spacing", 0.01);
			this->goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.03);
			this->goal_yaw_tolerance_ = this->declare_parameter<double>("goal_yaw_tolerance", 0.05);
			this->goal_settle_time_ = this->declare_parameter<double>("goal_settle_time", 0.2);
			this->goal_timeout_ = this->declare_parameter<double>("goal_timeout", 3.0);

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

			// routes_ はこれ以上増やさないので、要素を指しておいてよい
			this->active_ = &this->routes_.front().trajectory;

			this->reference_pub_ = this->create_publisher<TrackingReference>(reference_topic, rclcpp::QoS{10});
			// RViz で見る用。後から起動しても届くよう transient local
			this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS{1}.transient_local());
			this->publish_path();

			this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				odom_topic, rclcpp::QoS{10},
				[this](const nav_msgs::msg::Odometry::ConstSharedPtr m) {
					const auto& q = m->pose.pose.orientation;
					const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
					this->odom_ = Pose{m->pose.pose.position.x, m->pose.pose.position.y, yaw, this->now()};
				}
			);
			// ~/start は最初の経路、~/start/<名前> はその経路
			const auto add_start = [this](const std::string& service, const std::size_t index) {
				this->start_srvs_.push_back(this->create_service<std_srvs::srv::Trigger>(
					service,
					[this, index](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
						this->finish_goal(false, "preempted by ~/start");
						res->success = this->start(this->routes_[index].name, this->routes_[index].trajectory, res->message);
					}
				));
			};
			add_start("~/start", 0);
			for (std::size_t i = 0; i < this->routes_.size(); ++i) { add_start("~/start/" + this->routes_[i].name, i); }
			this->stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/stop",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					this->finish_goal(false, "stopped by ~/stop");
					this->running_ = false;
					res->success = true;
					res->message = "stopped";
					RCLCPP_INFO(this->get_logger(), "stopped");
				}
			);

			this->action_server_ = rclcpp_action::create_server<FollowPath>(
				this,
				"~/follow",
				[](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowPath::Goal> goal) {
					const bool ok = !goal->route.empty()
						|| (!goal->x.empty() && goal->x.size() == goal->y.size() && goal->x.size() == goal->yaw.size());
					return ok ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
				},
				[this](const std::shared_ptr<GoalHandle> handle) {
					if (handle == this->goal_handle_) {
						this->running_ = false;
					}
					return rclcpp_action::CancelResponse::ACCEPT;
				},
				[this](const std::shared_ptr<GoalHandle> handle) { this->on_accepted(handle); }
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
			double yaw{};
			rclcpp::Time received{};
		};

		/// 新しい自己位置があればそれを返す
		auto fresh_odom() -> std::optional<Pose> {
			if (!this->odom_ || (this->now() - this->odom_->received).seconds() > this->odom_timeout_) { return std::nullopt; }
			return this->odom_;
		}

		auto active() const -> const Trajectory& { return *this->active_; }

		/// trajectory の始点から走り出す。check_start なら、機体が始点から離れていれば始めない
		auto start(const std::string& name, const Trajectory& trajectory, std::string& message, const bool check_start = true) -> bool {
			// 始点から離れたところで始めると、追従ノードが始点へ一気に寄せようとする
			if (check_start && this->start_tolerance_ > 0.0) {
				const auto odom = this->fresh_odom();
				if (!odom) {
					message = "no recent pose estimate; not starting";
					RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
					return false;
				}
				const auto& first = trajectory.points().front();
				const double distance = std::hypot(odom->x - first.x, odom->y - first.y);
				if (distance > this->start_tolerance_) {
					message = "the robot is " + std::to_string(distance) + " m from the first point of path '" + name + "' (start_tolerance "
						+ std::to_string(this->start_tolerance_) + " m); not starting";
					RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
					return false;
				}
			}
			this->active_ = &trajectory;
			this->elapsed_ = 0.0;
			this->running_ = true;
			this->reported_goal_ = false;
			this->publish_path();
			message = "started '" + name + "'";
			RCLCPP_INFO(this->get_logger(), "started '%s' (%.2f m, %.1f s to the goal)", name.c_str(), trajectory.length(), trajectory.duration());
			return true;
		}

		// ---------------------------------------------------------------- アクション

		void on_accepted(const std::shared_ptr<GoalHandle> handle) {
			this->finish_goal(false, "preempted by a new goal");
			const auto goal = handle->get_goal();
			auto limits = this->limits_;
			if (goal->max_speed > 0.0) {
				limits.max_speed = std::min(limits.max_speed, goal->max_speed);
			}
			std::string message{};
			bool ok = false;
			try {
				if (!goal->route.empty()) {
					const auto it = std::ranges::find_if(this->routes_, [&](const Route& r) { return r.name == goal->route; });
					if (it == this->routes_.end()) {
						throw std::invalid_argument("no path named '" + goal->route + "'");
					}
					this->goal_trajectory_.emplace(it->trajectory.points(), limits);
					ok = this->start(goal->route, *this->goal_trajectory_, message);
				} else {
					const auto odom = this->fresh_odom();
					if (!odom) {
						throw std::invalid_argument("no recent pose estimate");
					}
					std::vector<path_follower::PathPoint> vertices{{odom->x, odom->y, odom->yaw}};
					for (std::size_t i = 0; i < goal->x.size(); ++i) {
						vertices.push_back({goal->x[i], goal->y[i], goal->yaw[i]});
					}
					// もう終点にいるなら走らない
					const auto& last = vertices.back();
					if (std::hypot(last.x - odom->x, last.y - odom->y) <= this->goal_tolerance_
						&& std::abs(std::remainder(last.yaw - odom->yaw, 2.0 * std::numbers::pi)) <= this->goal_yaw_tolerance_) {
						auto result = std::make_shared<FollowPath::Result>();
						result->success = true;
						result->message = "already at the goal";
						handle->succeed(result);
						return;
					}
					this->goal_trajectory_.emplace(path_follower::polyline_path(vertices, this->spacing_), limits);
					ok = this->start("waypoints", *this->goal_trajectory_, message, false);
				}
			} catch (const std::invalid_argument& e) {
				message = e.what();
			}
			if (!ok) {
				RCLCPP_WARN(this->get_logger(), "follow rejected: %s", message.c_str());
				auto result = std::make_shared<FollowPath::Result>();
				result->success = false;
				result->message = message;
				handle->abort(result);
				return;
			}
			this->goal_handle_ = handle;
			this->settled_since_.reset();
			this->finished_at_.reset();
		}

		/// アクションのゴールがあれば終わらせる
		void finish_goal(const bool success, const std::string& message) {
			if (!this->goal_handle_) {
				return;
			}
			if (this->goal_handle_->is_active()) {
				auto result = std::make_shared<FollowPath::Result>();
				result->success = success;
				result->message = message;
				if (success) {
					this->goal_handle_->succeed(result);
				} else if (this->goal_handle_->is_canceling()) {
					this->goal_handle_->canceled(result);
				} else {
					this->goal_handle_->abort(result);
				}
			}
			this->goal_handle_.reset();
		}

		/// 目標が終点に着いた後、機体が終点で止まったかを見る
		void update_goal(const State& state) {
			if (!this->goal_handle_) {
				return;
			}
			if (this->goal_handle_->is_canceling()) {
				this->finish_goal(false, "canceled");
				return;
			}
			const auto odom = this->fresh_odom();
			auto feedback = std::make_shared<FollowPath::Feedback>();
			feedback->remaining = this->active().length() - state.s;
			feedback->error = odom ? std::hypot(odom->x - state.x, odom->y - state.y) : -1.0;
			this->goal_handle_->publish_feedback(feedback);
			if (!state.finished) {
				return;
			}
			const auto now = this->now();
			if (!this->finished_at_) {
				this->finished_at_ = now;
			}
			const bool close = odom && std::hypot(odom->x - state.x, odom->y - state.y) <= this->goal_tolerance_
				&& std::abs(std::remainder(odom->yaw - state.yaw, 2.0 * std::numbers::pi)) <= this->goal_yaw_tolerance_;
			if (close) {
				if (!this->settled_since_) {
					this->settled_since_ = now;
				}
				if ((now - *this->settled_since_).seconds() >= this->goal_settle_time_) {
					this->finish_goal(true, "reached");
					return;
				}
			} else {
				this->settled_since_.reset();
			}
			if ((now - *this->finished_at_).seconds() > this->goal_timeout_) {
				char text[160];
				std::snprintf(text, sizeof(text), "did not settle at the goal within %.1f s (%.3f m away)", this->goal_timeout_,
					odom ? std::hypot(odom->x - state.x, odom->y - state.y) : -1.0);
				RCLCPP_WARN(this->get_logger(), "%s", text);
				this->finish_goal(false, text);
			}
		}

		void on_timer() {
			if (!this->running_) {
				if (this->goal_handle_) {
					this->finish_goal(false, "stopped");
				}
				return;
			}
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
			this->update_goal(state);
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
		/// いま走っている (最後に走った) 経路。routes_ の要素か goal_trajectory_
		const Trajectory* active_{nullptr};
		/// アクションで走る経路 (速さの上限を変えたもの、経由点をつないだもの)
		std::optional<Trajectory> goal_trajectory_{};
		Limits limits_{};
		double spacing_{};
		double goal_tolerance_{};
		double goal_yaw_tolerance_{};
		double goal_settle_time_{};
		double goal_timeout_{};
		std::shared_ptr<GoalHandle> goal_handle_{};
		std::optional<rclcpp::Time> settled_since_{};
		std::optional<rclcpp::Time> finished_at_{};
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
		rclcpp_action::Server<FollowPath>::SharedPtr action_server_{};
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
