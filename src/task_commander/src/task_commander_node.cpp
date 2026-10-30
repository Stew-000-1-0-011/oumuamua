/// 機体に作業を指示するノード。アクション `~/run` (task_commander/action/RunTask) で作業を 1 つずつ受け、
/// 走行 (path_follower の `~/follow`) と把持機構 (gripper_node の `~/set_lift_position`, `~/set_suction`) を順に動かす。
///
/// | 作業 | すること |
/// | --- | --- |
/// | PICK (gripper, block) | 昇降を clear に上げ、ブロックの手前 (pick.standoff) からブロックの真上へ寄り、
/// |                       | 昇降を pick に下ろして吸引をオン、carry に上げて、手前へ戻る |
/// | TO_SLALOM_ENTRANCE    | 昇降を carry にして、スラロームの入口へ (今の位置から直線) |
/// | TO_START              | 昇降を carry にして、スタート地点へ (今の位置から直線) |
/// | SLALOM_FORWARD        | 昇降を carry にして、経路 slalom.forward_route を走る (入口 -> 出口) |
/// | SLALOM_BACKWARD       | 昇降を carry にして、経路 slalom.backward_route を走る (出口 -> 入口) |
/// | PLACE (gripper, column, row) | 昇降をその段の approach に上げ、マスの手前 (shelf.standoff) からマスへ差し込み、
/// |                       | release に下ろして吸引をオフ、approach に戻して手前へ下がり、carry に下ろす |
/// | TO_SLALOM_EXIT        | 昇降を carry にして、スラロームの出口へ (今の位置から直線) |
///
/// 直線で動く前に、機体 (半径 robot_radius の円) が壁・棚・残っているブロックから min_clearance 以上
/// 離れているかを確かめ、近づくなら動かずに失敗を返す。
///
/// どの把持機構がブロックを持っているか、どのブロックを回収したか、どのマスに置いたかはこのノードが覚えている
/// (~/state に出す)。リトライなどで実際と合わなくなったら ~/reset で忘れさせる。

#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <expected>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gripper/msg/gripper_state.hpp>
#include <gripper/srv/set_lift_position.hpp>
#include <gripper/action/move_lift.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_follower/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "task_commander/action/run_task.hpp"
#include "task_commander/geometry.hpp"
#include "task_commander/pickup.hpp"
#include "task_commander/pickup_execution.hpp"

namespace {
	using RunTask = task_commander::action::RunTask;
	using GoalHandle = rclcpp_action::ServerGoalHandle<RunTask>;
	using FollowPath = path_follower::action::FollowPath;
	using MoveLift = gripper::action::MoveLift;
	using task_commander::Box;
	using task_commander::Field;
	using task_commander::Pose2;
	using namespace std::chrono_literals;
	auto steady_seconds() -> double {
		return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	/// 作業の途中の失敗 (取り消しを含む)
	using Status = std::expected<void, std::string>;

	struct Gripper final {
		std::string name;
		double offset_x{}; ///< 機体座標系での把持機構 (吸着面の中心) の位置 [m]
		double offset_y{};
		rclcpp::Client<gripper::srv::SetLiftPosition>::SharedPtr lift{};
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr suction{};
		rclcpp::Subscription<gripper::msg::GripperState>::SharedPtr state_sub{};
		std::optional<gripper::msg::GripperState> state{};
		bool holding{false};
		bool holding_uncertain{false};
		rclcpp_action::Client<MoveLift>::SharedPtr lift_action{};
		std::array<double,3> pickup_heights{}; // pick, clear, carry, resolved from gripper.yaml
		double pickup_tolerance{.005};
		std::uint64_t state_sequence{};
		double state_received_at{};
	};

	auto pose_from(const std::vector<double>& v, const char* name) -> Pose2 {
		if (v.size() != 3) {
			throw std::invalid_argument(std::string{name} + " must be [x, y, yaw]");
		}
		return Pose2{v[0], v[1], v[2]};
	}

	class TaskCommanderNode final : public rclcpp::Node {
	public:
		TaskCommanderNode() : rclcpp::Node("task_commander") {
			this->group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

			// --- フィールド (field.yaml の field.*) と機体 ---
			const auto bounds = this->declare_parameter<std::vector<double>>("field.bounds");
			auto walls = task_commander::boxes_from_flat(this->declare_parameter<std::vector<double>>("field.walls"), "field.walls");
			const auto shelf = task_commander::boxes_from_flat(this->declare_parameter<std::vector<double>>("field.shelf"), "field.shelf");
			if (bounds.size() != 4 || shelf.size() != 1) {
				throw std::invalid_argument("field.bounds and field.shelf must be [xmin, ymin, xmax, ymax]");
			}
			this->field_.bounds = Box{bounds[0], bounds[1], bounds[2], bounds[3]};
			this->field_.tall_obstacles = shelf;
			// 棚も壁と同じく避ける
			walls.push_back(shelf.front());
			this->walls_ = walls;
			this->robot_radius_ = this->declare_parameter<double>("robot_radius");
			this->min_clearance_ = this->declare_parameter<double>("min_clearance");

			// --- 場所 ---
			this->start_pose_ = pose_from(this->declare_parameter<std::vector<double>>("field.start_pose"), "field.start_pose");
			this->slalom_entrance_ = pose_from(this->declare_parameter<std::vector<double>>("slalom.entrance"), "slalom.entrance");
			this->slalom_exit_ = pose_from(this->declare_parameter<std::vector<double>>("slalom.exit"), "slalom.exit");
			this->forward_route_ = this->declare_parameter<std::string>("slalom.forward_route");
			this->backward_route_ = this->declare_parameter<std::string>("slalom.backward_route");

			// ノーツ (ブロック) は床に置かれた立方体。中心の x を並べる
			this->blocks_x_ = this->declare_parameter<std::vector<double>>("field.notes_x");
			this->blocks_y_ = this->declare_parameter<double>("field.notes_y");
			this->block_size_ = this->declare_parameter<double>("field.note_size");
			this->picked_.assign(this->blocks_x_.size(), false);
			this->uncertain_blocks_.assign(this->blocks_x_.size(), false);
			this->pick_yaw_ = this->declare_parameter<double>("pick.yaw");
			this->pick_standoff_ = this->declare_parameter<double>("pick.standoff");
			this->grip_time_ = this->declare_parameter<double>("pick.grip_time");
			this->pickup_settings_.yaw = this->pick_yaw_;
			this->pickup_settings_.standoff = this->pick_standoff_;
			this->pickup_settings_.margin = this->min_clearance_;
			this->pickup_settings_.yaw_offsets = this->declare_parameter<std::vector<double>>("pick.yaw_offsets");
			this->pickup_settings_.yaw_resolution = this->declare_parameter<double>("pick.yaw_resolution");
			this->pickup_settings_.transit_y = this->declare_parameter<double>("pick.transit_y");
			this->pickup_settings_.wall_padding = this->declare_parameter<double>("pick.wall_padding");
			this->pickup_settings_.bounds_padding = this->declare_parameter<double>("pick.bounds_padding");
			this->pickup_alignment_tolerance_ = this->declare_parameter<double>("pick.alignment_tolerance");
			this->pickup_require_feedback_ = this->declare_parameter<bool>("pick.require_feedback");
			const auto polygon = [](const std::vector<double>& flat) {
				if (flat.size() < 6 || flat.size() % 2) throw std::invalid_argument("pickup footprint needs x,y pairs");
				task_commander::Polygon out;
				for (std::size_t i=0;i<flat.size();i+=2) {
					if (!std::isfinite(flat[i]) || !std::isfinite(flat[i+1])) throw std::invalid_argument("pickup footprint must be finite");
					out.push_back({flat[i],flat[i+1]});
				}
				return out;
			};
			this->pickup_geometry_.raised = polygon(this->declare_parameter<std::vector<double>>("pick.raised_footprint"));
			this->pickup_geometry_.lowered = polygon(this->declare_parameter<std::vector<double>>("pick.lowered_footprint"));
			this->pickup_geometry_.contacts = task_commander::boxes_from_flat(this->declare_parameter<std::vector<double>>("pick.contact_regions"), "pick.contact_regions");
			this->pickup_layout_ = {this->blocks_x_,this->blocks_y_,this->block_size_};
			if (const auto valid=task_commander::validate_layout(this->pickup_layout_, {this->field_.bounds,this->walls_,this->field_.tall_obstacles}); !valid)
				throw std::invalid_argument(valid.error());
			if (!std::isfinite(this->grip_time_) || this->grip_time_<0 || !std::isfinite(this->pickup_alignment_tolerance_) || this->pickup_alignment_tolerance_<=0)
				throw std::invalid_argument("invalid pickup grip time or alignment tolerance");

			// 棚 (ビンゴ): 列の中心の x、自チーム側の面の y (棚の箱の ymax)、ブロックの中心を面からどれだけ奥に置くか
			this->columns_x_ = this->declare_parameter<std::vector<double>>("field.shelf_columns_x");
			this->rows_ = static_cast<int>(this->declare_parameter<std::int64_t>("field.shelf_rows"));
			this->shelf_face_y_ = shelf.front().ymax;
			this->place_depth_ = this->declare_parameter<double>("shelf.place_depth");
			this->place_yaw_ = this->declare_parameter<double>("place.yaw");
			this->place_standoff_ = this->declare_parameter<double>("place.standoff");
			this->release_time_ = this->declare_parameter<double>("place.release_time");
			this->filled_.assign(this->columns_x_.size() * static_cast<std::size_t>(std::max(this->rows_, 0)), false);

			// 昇降の高さは gripper.yaml の lift.positions の番号で指す
			this->lift_pick_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.pick"));
			this->lift_clear_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.clear"));
			this->lift_carry_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.carry"));
			const auto to_ints = [](const std::vector<std::int64_t>& v) { return std::vector<int>(v.begin(), v.end()); };
			this->lift_row_approach_ = to_ints(this->declare_parameter<std::vector<std::int64_t>>("lift.row_approach"));
			this->lift_row_release_ = to_ints(this->declare_parameter<std::vector<std::int64_t>>("lift.row_release"));
			if (std::ssize(this->lift_row_approach_) != this->rows_ || std::ssize(this->lift_row_release_) != this->rows_) {
				throw std::invalid_argument("lift.row_approach and lift.row_release need one index per shelf row");
			}
			this->lift_timeout_ = this->declare_parameter<double>("lift.timeout");
			if (!std::isfinite(this->lift_timeout_) || this->lift_timeout_<=0) throw std::invalid_argument("lift.timeout must be finite and positive");

			// 速さ: 寄るときと下がるとき (最後の一区間) は approach_speed
			this->travel_speed_ = this->declare_parameter<double>("travel_speed");
			this->approach_speed_ = this->declare_parameter<double>("approach_speed");
			this->pickup_motion_timeout_ = this->declare_parameter<double>("pick.motion_timeout");
			if (!std::isfinite(this->pickup_motion_timeout_) || this->pickup_motion_timeout_<=0
			    || !std::isfinite(this->travel_speed_) || this->travel_speed_<0
			    || !std::isfinite(this->approach_speed_) || this->approach_speed_<=0)
				throw std::invalid_argument("invalid pickup timeout or travel/approach speed");
			this->sweep_step_ = this->declare_parameter<double>("sweep_step");

			// --- 把持機構 ---
			const auto names = this->declare_parameter<std::vector<std::string>>("grippers.names");
			const auto offsets = this->declare_parameter<std::vector<double>>("grippers.offsets");
			if (offsets.size() != 2 * names.size()) {
				throw std::invalid_argument("grippers.offsets must be [x0, y0, x1, y1, ...] (two numbers per gripper)");
			}
			this->grippers_.resize(names.size());
			const auto pickup_heights = this->declare_parameter<std::vector<double>>("pick.lift_heights");
			const auto pickup_tolerances = this->declare_parameter<std::vector<double>>("pick.lift_tolerances");
			if (names.empty() || pickup_heights.size()!=3*names.size() || pickup_tolerances.size()!=names.size() || this->pickup_geometry_.contacts.size()!=names.size())
				throw std::invalid_argument("pickup heights/tolerances/contact regions must match grippers.names");
			for (std::size_t i = 0; i < names.size(); ++i) {
				auto& g = this->grippers_[i];
				g.name = names[i];
				g.offset_x = offsets[2 * i];
				g.offset_y = offsets[2 * i + 1];
				for (std::size_t j=0;j<3;++j) {
					g.pickup_heights[j]=pickup_heights[3*i+j];
					if (!std::isfinite(g.pickup_heights[j])) throw std::invalid_argument("pickup lift height must be finite");
				}
				g.pickup_tolerance=pickup_tolerances[i];
				if (!std::isfinite(g.pickup_tolerance) || g.pickup_tolerance<=0) throw std::invalid_argument("invalid pickup lift tolerance");
				g.lift_action = rclcpp_action::create_client<MoveLift>(this,"/"+g.name+"/move_lift",this->group_);
				g.lift = this->create_client<gripper::srv::SetLiftPosition>("/" + g.name + "/set_lift_position", rclcpp::ServicesQoS{}, this->group_);
				g.suction = this->create_client<std_srvs::srv::SetBool>("/" + g.name + "/set_suction", rclcpp::ServicesQoS{}, this->group_);
				rclcpp::SubscriptionOptions options{};
				options.callback_group = this->group_;
				g.state_sub = this->create_subscription<gripper::msg::GripperState>(
					"/" + g.name + "/state", rclcpp::QoS{10},
					[this, i](const gripper::msg::GripperState& m) {
						const std::scoped_lock lock{this->mutex_};
						this->grippers_[i].state = m;
						++this->grippers_[i].state_sequence;
						this->grippers_[i].state_received_at = steady_seconds();
					},
					options
				);
			}

			// --- 通信 ---
			const auto odom_topic = this->declare_parameter<std::string>("odom_topic");
			const auto follow_action = this->declare_parameter<std::string>("follow_action");
			rclcpp::SubscriptionOptions options{};
			options.callback_group = this->group_;
			this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				odom_topic, rclcpp::QoS{10},
				[this](const nav_msgs::msg::Odometry& m) {
					const auto& q = m.pose.pose.orientation;
					const std::scoped_lock lock{this->mutex_};
					this->odom_ = Pose2{
						m.pose.pose.position.x, m.pose.pose.position.y,
						std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
					};
					this->odom_stamp_ = this->now();
					this->odom_received_at_ = steady_seconds();
					this->odom_source_stamp_ = rclcpp::Time(m.header.stamp).nanoseconds();
					this->odom_in_field_ = m.header.frame_id == "field";
				},
				options
			);
			this->follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action, this->group_);
			this->state_pub_ = this->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS{1}.transient_local());
			this->reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reset",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					bool idle = false;
					if (!this->busy_.compare_exchange_strong(idle,true)) {
						res->success = false;
						res->message = "a task is running";
						return;
					}
					{
						const std::scoped_lock lock{this->mutex_};
						std::ranges::fill(this->picked_, false);
						std::ranges::fill(this->uncertain_blocks_, false);
						this->motion_stop_unconfirmed_=false;
						std::ranges::fill(this->filled_, false);
						for (auto& g : this->grippers_) {
							g.holding = false;
							g.holding_uncertain = false;
						}
					}
					this->publish_state();
					this->busy_ = false;
					res->success = true;
					res->message = "forgot picked blocks, held blocks and placed cells";
				},
				rclcpp::ServicesQoS{}, this->group_
			);
			this->action_server_ = rclcpp_action::create_server<RunTask>(
				this, "~/run",
				[this](const rclcpp_action::GoalUUID&, std::shared_ptr<const RunTask::Goal> goal) { return this->on_goal(*goal); },
				[](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
				[this](const std::shared_ptr<GoalHandle> handle) {
					std::thread{[this, handle] { this->execute(handle); }}.detach();
				},
				rcl_action_server_get_default_options(), this->group_
			);
			this->publish_state();
			RCLCPP_INFO(this->get_logger(), "ready: %zu grippers, %zu blocks, %zu x %d shelf cells",
				this->grippers_.size(), this->blocks_x_.size(), this->columns_x_.size(), this->rows_);
		}

	private:
		// ---------------------------------------------------------------- 受け付け

		auto check_goal(const RunTask::Goal& goal) -> std::optional<std::string> {
			const std::scoped_lock lock{this->mutex_};
			if (this->motion_stop_unconfirmed_) return "motion stop is unconfirmed; inspect and reconcile with ~/reset";
			for (const auto& g:this->grippers_) if(g.holding_uncertain) return "a load is uncertain; inspect and reconcile with ~/reset";
			const auto gripper_ok = goal.gripper >= 0 && goal.gripper < std::ssize(this->grippers_);
			switch (goal.type) {
			case RunTask::Goal::PICK:
				if (!gripper_ok) return std::format("gripper {} does not exist", goal.gripper);
				if (goal.block < 0 || goal.block >= std::ssize(this->blocks_x_)) return std::format("block {} does not exist", goal.block);
				return std::nullopt;
			case RunTask::Goal::PLACE:
				if (!gripper_ok) return std::format("gripper {} does not exist", goal.gripper);
				if (goal.column < 0 || goal.column >= std::ssize(this->columns_x_)) return std::format("column {} does not exist", goal.column);
				if (goal.row < 0 || goal.row >= this->rows_) return std::format("row {} does not exist", goal.row);
				return std::nullopt;
			case RunTask::Goal::TO_SLALOM_ENTRANCE:
			case RunTask::Goal::TO_START:
			case RunTask::Goal::SLALOM_FORWARD:
			case RunTask::Goal::SLALOM_BACKWARD:
			case RunTask::Goal::TO_SLALOM_EXIT:
				return std::nullopt;
			default:
				return std::format("unknown task type {}", goal.type);
			}
		}

		auto on_goal(const RunTask::Goal& goal) -> rclcpp_action::GoalResponse {
			if (const auto error = this->check_goal(goal)) {
				RCLCPP_WARN(this->get_logger(), "task rejected: %s", error->c_str());
				return rclcpp_action::GoalResponse::REJECT;
			}
			bool expected = false;
			if (!this->busy_.compare_exchange_strong(expected, true)) {
				RCLCPP_WARN(this->get_logger(), "task rejected: another task is running (cancel it first)");
				return rclcpp_action::GoalResponse::REJECT;
			}
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		// ---------------------------------------------------------------- 実行 (作業ごとのスレッド)

		void execute(const std::shared_ptr<GoalHandle> handle) {
			this->handle_ = handle;
			const auto& goal = *handle->get_goal();
			Status status{};
			switch (goal.type) {
			case RunTask::Goal::PICK: status = this->pick(goal.gripper, goal.block); break;
			case RunTask::Goal::TO_SLALOM_ENTRANCE: status = this->go_to("slalom entrance", this->slalom_entrance_); break;
			case RunTask::Goal::TO_START: status = this->go_to("start", this->start_pose_); break;
			case RunTask::Goal::SLALOM_FORWARD: status = this->slalom(this->forward_route_); break;
			case RunTask::Goal::SLALOM_BACKWARD: status = this->slalom(this->backward_route_); break;
			case RunTask::Goal::PLACE: status = this->place(goal.gripper, goal.column, goal.row); break;
			case RunTask::Goal::TO_SLALOM_EXIT: status = this->go_to("slalom exit", this->slalom_exit_); break;
			default: status = std::unexpected{"unknown task type"}; break;
			}

			auto result = std::make_shared<RunTask::Result>();
			result->success = status.has_value();
			result->message = status ? "done" : status.error();
			if (status) {
				RCLCPP_INFO(this->get_logger(), "task done");
				handle->succeed(result);
			} else if (handle->is_canceling()) {
				RCLCPP_WARN(this->get_logger(), "task canceled: %s", result->message.c_str());
				handle->canceled(result);
			} else {
				RCLCPP_ERROR(this->get_logger(), "task failed: %s", result->message.c_str());
				handle->abort(result);
			}
			this->publish_state();
			this->handle_.reset();
			this->busy_ = false;
		}

		void step(const std::string& what) {
			RCLCPP_INFO(this->get_logger(), "%s", what.c_str());
			auto feedback = std::make_shared<RunTask::Feedback>();
			feedback->step = what;
			this->handle_->publish_feedback(feedback);
		}

		auto canceled() const -> bool { return this->handle_ && this->handle_->is_canceling(); }

		/// 取り消されない間、until が真になるまで待つ
		auto wait(const std::function<bool()>& until, const double timeout, const std::string& what) -> Status {
			const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
			while (!until()) {
				if (this->canceled()) return std::unexpected{"canceled while " + what};
				if (!rclcpp::ok()) return std::unexpected{"shutting down"};
				if (std::chrono::steady_clock::now() > deadline) return std::unexpected{"timed out while " + what};
				std::this_thread::sleep_for(20ms);
			}
			return {};
		}

		auto sleep(const double seconds, const std::string& what) -> Status {
			const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
			return this->wait([end] { return std::chrono::steady_clock::now() >= end; }, seconds + 1.0, what);
		}

		template <class Service>
		auto call(const typename rclcpp::Client<Service>::SharedPtr& client, const typename Service::Request& request)
			-> std::expected<typename Service::Response, std::string> {
			if (!client->wait_for_service(1s)) {
				return std::unexpected{std::string{"service "} + client->get_service_name() + " is not available"};
			}
			auto future = client->async_send_request(std::make_shared<typename Service::Request>(request)).future.share();
			if (const auto s = this->wait([&] { return future.wait_for(0s) == std::future_status::ready; }, 2.0,
					std::string{"calling "} + client->get_service_name());
				!s) {
				return std::unexpected{s.error()};
			}
			return *future.get();
		}

		auto odom() -> std::optional<Pose2> {
			const std::scoped_lock lock{this->mutex_};
			if (!this->odom_ || (this->now() - this->odom_stamp_).seconds() > 0.5) return std::nullopt;
			return this->odom_;
		}

		auto pickup_odom() -> std::optional<Pose2> {
			const std::scoped_lock lock{this->mutex_};
			const auto source_age = this->now().nanoseconds()-this->odom_source_stamp_;
			if (!this->odom_ || !this->odom_in_field_ || source_age<0 || source_age>500000000
			    || steady_seconds()-this->odom_received_at_>.5) return std::nullopt;
			return this->odom_;
		}

		auto pickup_request(const int gripper,const int block,const Pose2& start) -> task_commander::PickupRequest {
			const std::scoped_lock lock{this->mutex_};
			const auto& g=this->grippers_[static_cast<std::size_t>(gripper)];
			task_commander::PickupRequest request{start,static_cast<std::size_t>(block),static_cast<std::size_t>(gripper),{g.offset_x,g.offset_y},this->picked_,{}};
			for (std::size_t i=0;i<this->picked_.size();++i) request.removed.push_back(this->picked_[i]&&!this->uncertain_blocks_[i]);
			return request;
		}

		// PICK uses the cancellable height action. Index service moves cannot be
		// cancelled; their heights are resolved from the same gripper YAML at launch.
		auto set_pickup_lift(const int gripper,const std::size_t slot) -> Status {
			auto& g=this->grippers_[static_cast<std::size_t>(gripper)];
			auto client=g.lift_action;
			if (!client->wait_for_action_server(1s)) return std::unexpected{g.name+": move_lift is unavailable"};
			if (this->canceled()) return std::unexpected{"canceled before lift request"};
			std::uint64_t before;
			{ const std::scoped_lock lock{this->mutex_}; before=g.state_sequence; }
			const auto issued=this->now().nanoseconds();
			MoveLift::Goal goal{};goal.height=g.pickup_heights.at(slot);
			auto abandoned=std::make_shared<std::atomic<bool>>(false);
			rclcpp_action::Client<MoveLift>::SendGoalOptions options;
			options.goal_response_callback=[client,abandoned](const rclcpp_action::ClientGoalHandle<MoveLift>::SharedPtr& h) {
				if (h&&abandoned->load()) client->async_cancel_goal(h);
			};
			auto future=client->async_send_goal(goal,options);
			if (auto s=this->wait([&]{return future.wait_for(0s)==std::future_status::ready;},2.,"sending lift goal"); !s) {
				abandoned->store(true);
				this->motion_stop_unconfirmed_=true;
				if (future.wait_for(0s)==std::future_status::ready) { if(auto h=future.get()) client->async_cancel_goal(h); }
				return s;
			}
			const auto handle=future.get();
			if (!handle) return std::unexpected{g.name+": lift goal rejected"};
			auto result=client->async_get_result(handle);
			const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(this->lift_timeout_);
			while (result.wait_for(20ms)!=std::future_status::ready) {
				if (this->canceled() || !rclcpp::ok() || std::chrono::steady_clock::now()>=deadline) {
					client->async_cancel_goal(handle);
					// Wait briefly for the gripper's stop_move acknowledgement, without
					// ignoring it merely because the parent action is already cancelling.
					if (result.wait_for(2s)!=std::future_status::ready)
						{ this->motion_stop_unconfirmed_=true; return std::unexpected{g.name+": lift cancel unconfirmed; inspect mechanism"}; }
					return std::unexpected{g.name+": lift canceled or timed out"};
				}
			}
			const auto wrapped=result.get();
			if (wrapped.code!=rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result || !wrapped.result->reached
			    || !std::isfinite(wrapped.result->height) || std::abs(wrapped.result->height-goal.height)>g.pickup_tolerance)
				return std::unexpected{g.name+": "+(wrapped.result?wrapped.result->message:"no lift result")};
			return this->wait([&] {
				const std::scoped_lock lock{this->mutex_};
				if (!g.state) return false;
				const auto stamp=rclcpp::Time(g.state->header.stamp).nanoseconds();
				const task_commander::LiftObservation observation{g.state_sequence,stamp,g.state_received_at,g.state->lift_height,
				    g.state->lift_arrived,g.state->lift_moving,g.state->lift_feedback};
				return stamp<=this->now().nanoseconds()&&task_commander::pickup_lift_confirmed(observation,before,issued,
				    goal.height,steady_seconds(),this->pickup_require_feedback_,g.pickup_tolerance);
			},1.,g.name+": waiting for fresh lift arrival feedback");
		}

		auto move_pickup(const std::vector<Pose2>& poses,const int gripper,const int block,const bool lowered,const bool captured,const double speed) -> Status {
			if(auto ready=this->pickup_lifts_ready(gripper,lowered);!ready)return ready;
			const auto here=this->pickup_odom();
			if (!here) return std::unexpected{"no fresh field-frame odometry for pickup"};
			std::vector<Pose2> sweep{*here};
			for (const auto& p:poses) {
				if (std::hypot(p.x-sweep.back().x,p.y-sweep.back().y)<=1e-6) {
					if (std::abs(std::remainder(p.yaw-sweep.back().yaw,2*std::numbers::pi))>1e-6)
						return std::unexpected{"pickup path would require an unsupported in-place turn"};
				} else sweep.push_back(p);
			}
			const auto request=this->pickup_request(gripper,block,*here);
			const auto gap=task_commander::pickup_stage_clearance(this->pickup_layout_,{this->field_.bounds,this->walls_,this->field_.tall_obstacles},
			    this->pickup_geometry_,this->pickup_settings_,request,sweep,lowered,captured);
			if (gap<this->min_clearance_) return std::unexpected{std::format("pickup stage clearance {:.3f} m below {:.3f} m",gap,this->min_clearance_)};
			if (sweep.size()==1) return {};
			FollowPath::Goal goal{};goal.max_speed=speed;
			for (std::size_t i=1;i<sweep.size();++i) { goal.x.push_back(sweep[i].x);goal.y.push_back(sweep[i].y);goal.yaw.push_back(sweep[i].yaw); }
			return this->follow(goal,"pickup movement",this->pickup_motion_timeout_,[this,gripper,lowered]()->Status {
				if(!this->pickup_odom())return std::unexpected{"pickup odometry became stale"};
				return this->pickup_lifts_ready(gripper,lowered);
			});
		}

		auto pickup_lifts_ready(const int selected,const bool lowered) -> Status {
			const std::scoped_lock lock{this->mutex_};
			for(std::size_t i=0;i<this->grippers_.size();++i) {
				const auto& g=this->grippers_[i];
				const auto slot=(static_cast<int>(i)==selected&&lowered)?1:2;
				if(!g.state)return std::unexpected{g.name+": no pickup lift state"};
				const auto stamp=rclcpp::Time(g.state->header.stamp).nanoseconds();
				const auto age=this->now().nanoseconds()-stamp;
				const task_commander::LiftObservation o{g.state_sequence,stamp,g.state_received_at,g.state->lift_height,
				    g.state->lift_arrived,g.state->lift_moving,g.state->lift_feedback};
				if(age<0||age>500000000||!task_commander::pickup_lift_confirmed(o,0,0,g.pickup_heights[slot],
				    steady_seconds(),this->pickup_require_feedback_,g.pickup_tolerance))
					return std::unexpected{g.name+": pickup lift left its confirmed height or feedback became stale"};
			}
			return {};
		}

		auto check_pickup_lowering(const int gripper,const int block,const bool align) -> Status {
			const auto pose=this->pickup_odom();
			if (!pose) return std::unexpected{"no fresh field pose before lowering"};
			const auto request=this->pickup_request(gripper,block,*pose);
			const double gap=task_commander::pickup_stage_clearance(this->pickup_layout_,{this->field_.bounds,this->walls_,this->field_.tall_obstacles},
			    this->pickup_geometry_,this->pickup_settings_,request,{*pose},true,false);
			if (gap<this->min_clearance_) return std::unexpected{"actual pose is unsafe for lowering"};
			if (align) {
				const auto tool=task_commander::offset_pose(*pose,request.tool[0],request.tool[1]);
				if (std::hypot(tool.x-this->blocks_x_[static_cast<std::size_t>(block)],tool.y-this->blocks_y_)>this->pickup_alignment_tolerance_)
					return std::unexpected{"suction centre is not aligned with box; not lowering"};
			}
			return {};
		}

		// ---------------------------------------------------------------- 昇降と吸引

		/// gripper の昇降を lift.positions の index 番目へ。wait_arrival なら着くまで待つ
		auto set_lift(const int gripper, const int index, const bool wait_arrival = true) -> Status {
			auto& g = this->grippers_[static_cast<std::size_t>(gripper)];
			gripper::srv::SetLiftPosition::Request request{};
			request.index = index;
			const auto response = this->call<gripper::srv::SetLiftPosition>(g.lift, request);
			if (!response) return std::unexpected{response.error()};
			if (!response->success) return std::unexpected{g.name + ": " + response->message};
			if (!wait_arrival) return {};
			return this->wait(
				[&] {
					const std::scoped_lock lock{this->mutex_};
					return g.state && g.state->lift_position == index && g.state->lift_arrived;
				},
				this->lift_timeout_, std::format("waiting for {} to reach lift position {}", g.name, index)
			);
		}

		/// 全部の把持機構の昇降を同時に動かし、全部着くまで待つ
		auto set_all_lifts(const int index) -> Status {
			for (std::size_t i = 0; i < this->grippers_.size(); ++i) {
				if (const auto s = this->set_lift(static_cast<int>(i), index, false); !s) return s;
			}
			for (std::size_t i = 0; i < this->grippers_.size(); ++i) {
				if (const auto s = this->set_lift(static_cast<int>(i), index, true); !s) return s;
			}
			return {};
		}

		auto set_suction(const int gripper, const bool on) -> Status {
			auto& g = this->grippers_[static_cast<std::size_t>(gripper)];
			std_srvs::srv::SetBool::Request request{};
			request.data = on;
			const auto response = this->call<std_srvs::srv::SetBool>(g.suction, request);
			if (!response) return std::unexpected{response.error()};
			if (!response->success) return std::unexpected{g.name + ": " + response->message};
			return {};
		}

		// ---------------------------------------------------------------- 走行

		/// 壁・棚と、まだ回収していないブロック
		auto obstacles() -> Field {
			Field field{.bounds = this->field_.bounds, .obstacles = this->walls_};
			const std::scoped_lock lock{this->mutex_};
			const double h = 0.5 * this->block_size_;
			for (std::size_t i = 0; i < this->blocks_x_.size(); ++i) {
				if (!this->picked_[i] || this->uncertain_blocks_[i]) {
					field.obstacles.push_back(Box{this->blocks_x_[i] - h, this->blocks_y_ - h, this->blocks_x_[i] + h, this->blocks_y_ + h});
				}
			}
			return field;
		}

		auto follow(const FollowPath::Goal& goal, const std::string& what, const double timeout=0.,const std::function<Status()>& guard={}) -> Status {
			if (!this->follow_client_->wait_for_action_server(1s)) return std::unexpected{"path_follower's follow action is not available"};
			auto abandoned=std::make_shared<std::atomic<bool>>(false);
			rclcpp_action::Client<FollowPath>::SendGoalOptions options;
			options.goal_response_callback=[client=this->follow_client_,abandoned](const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr& handle) {
				if(handle&&abandoned->load())client->async_cancel_goal(handle);
			};
			auto goal_future = this->follow_client_->async_send_goal(goal,options);
			if (const auto s = this->wait([&] { return goal_future.wait_for(0s) == std::future_status::ready; }, 2.0, "sending " + what); !s) {
				abandoned->store(true);
				if(timeout>0)this->motion_stop_unconfirmed_=true;
				if(goal_future.wait_for(0s)==std::future_status::ready) { if(auto h=goal_future.get())this->follow_client_->async_cancel_goal(h); }
				return s;
			}
			const auto goal_handle = goal_future.get();
			if (!goal_handle) return std::unexpected{"path_follower rejected " + what};
			auto result_future = this->follow_client_->async_get_result(goal_handle);
			const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout>0?timeout:1e9);
			while (result_future.wait_for(20ms) != std::future_status::ready) {
				if(guard)if(auto status=guard();!status) {
					this->follow_client_->async_cancel_goal(goal_handle);
					if(result_future.wait_for(2s)!=std::future_status::ready)this->motion_stop_unconfirmed_=true;
					return status;
				}
				if (this->canceled() || !rclcpp::ok() || std::chrono::steady_clock::now()>deadline) {
					this->follow_client_->async_cancel_goal(goal_handle);
					if(timeout>0 && result_future.wait_for(2s)!=std::future_status::ready) this->motion_stop_unconfirmed_=true;
					return std::unexpected{"canceled while " + what};
				}
			}
			const auto wrapped = result_future.get();
			if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result || !wrapped.result->success) {
				return std::unexpected{what + ": " + (wrapped.result ? wrapped.result->message : std::string{"no result"})};
			}
			return {};
		}

		/// 今の位置から poses を順に直線でたどる。前もって壁などとの間隔を確かめる。
		/// 最後の区間だけ last_speed (0 なら travel_speed) で動く
		auto move(const std::vector<Pose2>& poses, const std::string& what, const double last_speed = 0.0) -> Status {
			const auto here = this->odom();
			if (!here) return std::unexpected{"no recent pose estimate"};
			std::vector<Pose2> sweep{*here};
			sweep.insert(sweep.end(), poses.begin(), poses.end());
			const auto worst = task_commander::sweep_clearance(this->obstacles(), sweep, this->robot_radius_, this->sweep_step_);
			if (worst.clearance < this->min_clearance_) {
				return std::unexpected{std::format("{}: the robot would come within {:.3f} m of an obstacle at ({:.3f}, {:.3f}) (min_clearance {:.3f}); not moving",
					what, worst.clearance, worst.x, worst.y, this->min_clearance_)};
			}
			// 速さの違う区間は分けて走る
			const auto run = [&](const std::vector<Pose2>& part, const double speed) -> Status {
				if (part.empty()) return {};
				FollowPath::Goal goal{};
				for (const auto& p : part) {
					goal.x.push_back(p.x);
					goal.y.push_back(p.y);
					goal.yaw.push_back(p.yaw);
				}
				goal.max_speed = speed;
				return this->follow(goal, what);
			};
			if (last_speed > 0.0 && !poses.empty()) {
				if (const auto s = run({poses.begin(), poses.end() - 1}, this->travel_speed_); !s) return s;
				return run({poses.back()}, last_speed);
			}
			return run(poses, this->travel_speed_);
		}

		// ---------------------------------------------------------------- 作業

		auto go_to(const std::string& name, const Pose2& pose) -> Status {
			this->step("lifts to carry");
			if (const auto s = this->set_all_lifts(this->lift_carry_); !s) return s;
			this->step("moving to the " + name);
			return this->move({pose}, "moving to the " + name);
		}

		auto slalom(const std::string& route) -> Status {
			this->step("lifts to carry");
			if (const auto s = this->set_all_lifts(this->lift_carry_); !s) return s;
			this->step("following " + route);
			FollowPath::Goal goal{};
			goal.route = route;
			goal.max_speed = this->travel_speed_;
			return this->follow(goal, "following " + route);
		}

		auto pick(const int gripper, const int block) -> Status {
			auto& g=this->grippers_[static_cast<std::size_t>(gripper)];
			{
				const std::scoped_lock lock{this->mutex_};
				if(g.holding || g.holding_uncertain)return std::unexpected{g.name+" is holding or uncertain; reconcile and reset first"};
				if(this->picked_[static_cast<std::size_t>(block)])return std::unexpected{"box is held or uncertain; reconcile and reset first"};
			}
			const auto here=this->pickup_odom();
			if(!here)return std::unexpected{"no fresh field-frame pose for pickup"};
			const auto plan=task_commander::plan_pickup(this->pickup_layout_,{this->field_.bounds,this->walls_,this->field_.tall_obstacles},
			    this->pickup_geometry_,this->pickup_settings_,this->pickup_request(gripper,block,*here));
			if(!plan)return std::unexpected{plan.error()};
			this->step(std::format("{}: pickup plan clearance {:.3f} m",g.name,plan->clearance));
			const auto result=task_commander::run_pickup([&](task_commander::PickupStage stage)->Status {
				using Stage=task_commander::PickupStage;
				switch(stage) {
				case Stage::RaiseAll:
					this->step("all lifts to carry before travel");
					for(std::size_t i=0;i<this->grippers_.size();++i)if(auto status=this->set_pickup_lift(static_cast<int>(i),2);!status)return status;
					return {};
				case Stage::Transit:
					this->step("pickup: raised transit to standoff");
					return this->move_pickup(plan->transit,gripper,block,false,false,this->travel_speed_);
				case Stage::Clear:
					this->step("pickup: selected lift to clear");
					if(auto status=this->check_pickup_lowering(gripper,block,false);!status)return status;
					return this->set_pickup_lift(gripper,1);
				case Stage::Approach:
					this->step("pickup: lowered final approach");
					return this->move_pickup(plan->approach,gripper,block,true,false,this->approach_speed_);
				case Stage::Lower:
					this->step("pickup: align suction centre, lower to pick");
					if(auto status=this->check_pickup_lowering(gripper,block,true);!status)return status;
					return this->set_pickup_lift(gripper,0);
				case Stage::Suction:
					this->step("pickup: suction requested; load uncertain until raised");
					return this->set_suction(gripper,true);
				case Stage::GripWait:return this->sleep(this->grip_time_,"gripping");
				case Stage::Carry:
					this->step("pickup: raise and confirm fresh carry feedback");
					return this->set_pickup_lift(gripper,2);
				case Stage::Retreat:
					this->step("pickup: raised retreat");
					return this->move_pickup(plan->retreat,gripper,block,false,true,this->approach_speed_);
				}
				return std::unexpected{"invalid pickup stage"};
			},[&](task_commander::PickupLoad load) {
				{
					const std::scoped_lock lock{this->mutex_};
					g.holding=true;
					g.holding_uncertain=load==task_commander::PickupLoad::Uncertain;
					this->picked_[static_cast<std::size_t>(block)]=true;
					this->uncertain_blocks_[static_cast<std::size_t>(block)]=g.holding_uncertain;
				}
				this->publish_state();
			},[&]()->Status {
				if(this->canceled()||!rclcpp::ok())return std::unexpected{"pickup canceled"};
				return {};
			});
			if(!result&&g.holding_uncertain)return std::unexpected{result.error()+"; load uncertain: inspect box/gripper and reconcile with ~/reset"};
			return result;
		}

		auto place(const int gripper, const int column, const int row) -> Status {
			auto& g = this->grippers_[static_cast<std::size_t>(gripper)];
			const auto cell = static_cast<std::size_t>(column * this->rows_ + row);
			{
				const std::scoped_lock lock{this->mutex_};
				if (!g.holding) return std::unexpected{g.name + " is not holding a block"};
				if (g.holding_uncertain) return std::unexpected{g.name + " load is uncertain; reconcile and reset before PLACE"};
				if (this->filled_[cell]) return std::unexpected{std::format("cell (column {}, row {}) already has our block", column, row)};
				if (row > 0 && !this->filled_[cell - 1]) {
					// 2 段目より上は、同じ列の下の段にノーツがあるときだけ (相手のノーツでもよいので、ここでは止めない)
					RCLCPP_WARN(this->get_logger(), "column %d row %d: we have not placed below it; make sure some note is there (rule 2.5.2)", column, row);
				}
			}
			const auto ri = static_cast<std::size_t>(row);
			const auto target = task_commander::pose_for_tool(
				this->columns_x_[static_cast<std::size_t>(column)], this->shelf_face_y_ - this->place_depth_, this->place_yaw_, g.offset_x, g.offset_y);
			const double reach = std::hypot(g.offset_x, g.offset_y);
			const auto standoff = task_commander::offset_pose(target, -this->place_standoff_ * g.offset_x / reach, -this->place_standoff_ * g.offset_y / reach);

			this->step(std::format("{}: lift to row {} approach", g.name, row));
			if (const auto s = this->set_lift(gripper, this->lift_row_approach_[ri]); !s) return s;
			this->step(std::format("moving {} into column {} row {}", g.name, column, row));
			if (const auto s = this->move({standoff, target}, std::format("moving {} into column {} row {}", g.name, column, row), this->approach_speed_); !s) return s;
			this->step(std::format("{}: down, suction off", g.name));
			if (const auto s = this->set_lift(gripper, this->lift_row_release_[ri]); !s) return s;
			if (const auto s = this->set_suction(gripper, false); !s) return s;
			{
				const std::scoped_lock lock{this->mutex_};
				g.holding = false;
				this->filled_[cell] = true;
			}
			this->publish_state();
			if (const auto s = this->sleep(this->release_time_, "releasing"); !s) return s;
			this->step(std::format("{}: up, backing off", g.name));
			if (const auto s = this->set_lift(gripper, this->lift_row_approach_[ri]); !s) return s;
			if (const auto s = this->move({standoff}, "backing off from the shelf", this->approach_speed_); !s) return s;
			this->step(std::format("{}: lift to carry", g.name));
			return this->set_lift(gripper, this->lift_carry_);
		}

		// ---------------------------------------------------------------- 状態

		void publish_state() {
			std::string text{};
			{
				const std::scoped_lock lock{this->mutex_};
				text += "holding:";
				for (const auto& g : this->grippers_) {
					text += std::format(" {}={}", g.name, g.holding ? "yes" : "no");
				}
				text += "\npicked blocks:";
				for (std::size_t i = 0; i < this->picked_.size(); ++i) {
					if (this->picked_[i]) text += std::format(" {}", i);
				}
				text += "\nplaced cells (column, row):";
				for (std::size_t c = 0; c < this->columns_x_.size(); ++c) {
					for (int r = 0; r < this->rows_; ++r) {
						if (this->filled_[c * static_cast<std::size_t>(this->rows_) + static_cast<std::size_t>(r)]) text += std::format(" ({}, {})", c, r);
					}
				}
				text += "\nuncertain grippers:";
				for (const auto& g:this->grippers_) if(g.holding_uncertain) text += " "+g.name;
			}
			std_msgs::msg::String m{};
			m.data = text;
			this->state_pub_->publish(m);
		}

		rclcpp::CallbackGroup::SharedPtr group_{};
		std::mutex mutex_{};
		std::atomic<bool> busy_{false};
		std::shared_ptr<GoalHandle> handle_{};

		Field field_{};
		std::vector<Box> walls_{};
		double robot_radius_{};
		double min_clearance_{};
		Pose2 start_pose_{};
		Pose2 slalom_entrance_{};
		Pose2 slalom_exit_{};
		std::string forward_route_{};
		std::string backward_route_{};
		std::vector<double> blocks_x_{};
		double blocks_y_{};
		double block_size_{};
		double pick_yaw_{};
		double pick_standoff_{};
		double grip_time_{};
		std::vector<double> columns_x_{};
		int rows_{};
		double shelf_face_y_{};
		double place_depth_{};
		double place_yaw_{};
		double place_standoff_{};
		double release_time_{};
		int lift_pick_{};
		int lift_clear_{};
		int lift_carry_{};
		std::vector<int> lift_row_approach_{};
		std::vector<int> lift_row_release_{};
		double lift_timeout_{};
		double travel_speed_{};
		double approach_speed_{};
		double sweep_step_{};
		std::vector<Gripper> grippers_{};

		// 覚えていること (mutex_)
		std::vector<bool> picked_{};
		std::vector<bool> uncertain_blocks_{};
		task_commander::PickupLayout pickup_layout_{};
		task_commander::PickupGeometry pickup_geometry_{};
		task_commander::PickupSettings pickup_settings_{};
		bool pickup_require_feedback_{true};
		std::atomic<bool> motion_stop_unconfirmed_{false};
		double pickup_motion_timeout_{30.};
		double pickup_alignment_tolerance_{.03};
		std::vector<bool> filled_{};
		std::optional<Pose2> odom_{};
		rclcpp::Time odom_stamp_{};
		double odom_received_at_{};
		std::int64_t odom_source_stamp_{};
		bool odom_in_field_{};

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
		rclcpp_action::Client<FollowPath>::SharedPtr follow_client_{};
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_{};
		rclcpp_action::Server<RunTask>::SharedPtr action_server_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 4};
	try {
		auto node = std::make_shared<TaskCommanderNode>();
		executor.add_node(node);
		executor.spin();
	} catch (const std::exception& e) {
		RCLCPP_FATAL(rclcpp::get_logger("task_commander"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
