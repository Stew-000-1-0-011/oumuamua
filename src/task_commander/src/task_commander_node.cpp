/// 機体に作業を指示するノード。アクション `~/run` (task_commander/action/RunTask) で作業を 1 つずつ受け、
/// 走行 (path_follower の `~/follow`) と把持機構 (gripper_node の `~/set_lift_position`, `~/set_suction`) を順に動かす。
///
/// | 作業 | すること |
/// | --- | --- |
/// | PICK (gripper, block) | 昇降を clear に上げて吸引をオン、経由点 (pick.via) と手前 (pick.standoff) を通って、回収の姿勢
/// |                       | (pick.poses) へ寄り、昇降を pick に下ろしきって grip_time 待ち、carry に上げて、手前へ戻る |
/// | PICK_PAIR (block)     | 青なら gripper0 で block、gripper1 で block - 1 を (赤なら左右が逆)、同時に回収する。姿勢は pick.pairs。手順は PICK と同じ |
/// | TO_SLALOM_ENTRANCE    | 昇降を carry にして、スラロームの入口へ (今の位置から直線) |
/// | TO_START              | 昇降を carry にして、スタート地点へ (今の位置から直線) |
/// | SLALOM_FORWARD        | 昇降を carry にして、経路 slalom.forward_route を走る (入口 -> 出口) |
/// | SLALOM_BACKWARD       | 昇降を carry にして、経路 slalom.backward_route を走る (出口 -> 入口) |
/// | TO_SHELF_CELL (gripper, column) | 今の y のまま、把持機構 gripper の吸ったノーツの x が列 column の中心に来るように横へ動き、
/// |                       | 棚の方 (place.yaw) を向く。前後 (棚へ差し込む)・昇降・吸引は手で |
/// | TO_SLALOM_EXIT        | 昇降を carry にして、スラロームの出口へ (今の位置から直線) |
/// | TO_SHELF_FRONT        | 昇降を carry にして、棚の前 (中央の列の正面、吸ったノーツの前の面が棚の中心面から
/// |                       | shelf.front_distance) へ (今の位置から直線) |
///
/// 直線で動く前に、機体 (外形 footprint の多角形。持ち上げた把持機構は入れない) が壁・棚・残っているブロックから
/// min_clearance 以上離れているかを確かめ、近づくなら動かずに失敗を返す。
/// 回収を始めるときの機体の姿勢は、ブロックと把持機構ごとに pick.poses で決める。
///
/// どの把持機構がブロックを持っているか、どのブロックを回収したかはこのノードが覚えている (~/state に出す)。
/// 持っている把持機構の吸引が切れたら (手で置いたら)、持っていないことにする。
/// リトライなどで実際と合わなくなったら ~/reset で忘れさせる。

#include <algorithm>
#include <array>
#include <map>
#include <atomic>
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
#include <tuple>
#include <utility>
#include <vector>

#include <gripper/msg/gripper_state.hpp>
#include <gripper/srv/set_lift_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_follower/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "task_commander/action/run_task.hpp"
#include "task_commander/geometry.hpp"

namespace {
	using RunTask = task_commander::action::RunTask;
	using GoalHandle = rclcpp_action::ServerGoalHandle<RunTask>;
	using FollowPath = path_follower::action::FollowPath;
	using task_commander::Box;
	using task_commander::Field;
	using task_commander::Pose2;
	using namespace std::chrono_literals;

	/// 作業の途中の失敗 (取り消しを含む)
	using Status = std::expected<void, std::string>;

	struct Gripper final {
		std::string name;
		double offset_x{}; ///< 機体座標系での、吸ったノーツの中心の位置 [m] (吸着面の中心 + ノーツの大きさの半分だけ前)
		double offset_y{};
		std::vector<Pose2> pick_poses{}; ///< ブロックごとの、回収を始めるときの機体の姿勢
		rclcpp::Client<gripper::srv::SetLiftPosition>::SharedPtr lift{};
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr suction{};
		rclcpp::Subscription<gripper::msg::GripperState>::SharedPtr state_sub{};
		std::optional<gripper::msg::GripperState> state{};
		bool holding{false};
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

			// --- 自チームの色 ---
			// このノードは、自チームが +y 側にある座標 (青コートの座標。以下「自チームの座標」) で考える。
			// 赤 (-y 側) なら、フィールドの形・機体の左右を y について反転して取り込み、自己位置を受け取るときと
			// path_follower へ目標を送るときに反転する。反転した世界では機体も鏡写しなので、左右の把持機構の役が入れ替わる
			const auto team = this->declare_parameter<std::string>("field.team");
			if (team != "blue" && team != "red") {
				throw std::invalid_argument("field.team must be blue or red");
			}
			this->red_ = team == "red";
			const auto mirror_box = [this](const Box& b) { return this->red_ ? Box{b.xmin, -b.ymax, b.xmax, -b.ymin} : b; };

			// --- フィールド (field.yaml の field.*) と機体 ---
			const auto bounds = this->declare_parameter<std::vector<double>>("field.bounds");
			auto walls = task_commander::boxes_from_flat(this->declare_parameter<std::vector<double>>("field.walls"), "field.walls");
			const auto shelf = task_commander::boxes_from_flat(this->declare_parameter<std::vector<double>>("field.shelf"), "field.shelf");
			if (bounds.size() != 4 || shelf.size() != 1) {
				throw std::invalid_argument("field.bounds and field.shelf must be [xmin, ymin, xmax, ymax]");
			}
			this->field_.bounds = mirror_box(Box{bounds[0], bounds[1], bounds[2], bounds[3]});
			// 棚も壁と同じく避ける
			walls.push_back(shelf.front());
			for (auto& w : walls) {
				w = mirror_box(w);
			}
			this->walls_ = walls;
			this->footprint_ = task_commander::footprint_from_flat(this->declare_parameter<std::vector<double>>("footprint"), "footprint");
			if (this->red_) {
				for (auto& p : this->footprint_) {
					p.y = -p.y;
				}
			}
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
			this->pick_via_ = pose_from(this->declare_parameter<std::vector<double>>("pick.via"), "pick.via");
			this->pick_standoff_ = this->declare_parameter<double>("pick.standoff");
			this->pick_red_extra_reach_ = this->declare_parameter<double>("pick.red_extra_reach");
			this->grip_time_ = this->declare_parameter<double>("pick.grip_time");

			// 棚 (ビンゴ): 列の中心の x と、棚の中心面の y
			this->columns_x_ = this->declare_parameter<std::vector<double>>("field.shelf_columns_x");
			this->rows_ = static_cast<int>(this->declare_parameter<std::int64_t>("field.shelf_rows"));
			this->shelf_center_y_ = 0.5 * (shelf.front().ymin + shelf.front().ymax);
			const double front_distance = this->declare_parameter<double>("shelf.front_distance");
			this->place_yaw_ = this->declare_parameter<double>("place.yaw");

			// 昇降の高さは gripper.yaml の lift.positions の番号で指す
			this->lift_pick_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.pick"));
			this->lift_clear_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.clear"));
			this->lift_carry_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.carry"));
			this->lift_timeout_ = this->declare_parameter<double>("lift.timeout");

			// 速さ: 寄るときと下がるとき (最後の一区間) は approach_speed
			this->travel_speed_ = this->declare_parameter<double>("travel_speed");
			this->approach_speed_ = this->declare_parameter<double>("approach_speed");
			this->sweep_step_ = this->declare_parameter<double>("sweep_step");

			// --- 把持機構 ---
			const auto names = this->declare_parameter<std::vector<std::string>>("grippers.names");
			const auto suction = this->declare_parameter<std::vector<double>>("grippers.suction_offsets");
			if (suction.size() != 2 * names.size()) {
				throw std::invalid_argument("grippers.suction_offsets must be [x0, y0, x1, y1, ...] (two numbers per gripper)");
			}
			this->grippers_.resize(names.size());
			for (std::size_t i = 0; i < names.size(); ++i) {
				auto& g = this->grippers_[i];
				g.name = names[i];
				// 吸着面は機体の前を向いているので、吸ったノーツの中心は吸着面の中心からノーツの大きさの半分だけ前
				g.offset_x = suction[2 * i] + 0.5 * this->block_size_;
				g.offset_y = this->red_ ? -suction[2 * i + 1] : suction[2 * i + 1];
				g.lift = this->create_client<gripper::srv::SetLiftPosition>("/" + g.name + "/set_lift_position", rclcpp::ServicesQoS{}, this->group_);
				g.suction = this->create_client<std_srvs::srv::SetBool>("/" + g.name + "/set_suction", rclcpp::ServicesQoS{}, this->group_);
				rclcpp::SubscriptionOptions options{};
				options.callback_group = this->group_;
				g.state_sub = this->create_subscription<gripper::msg::GripperState>(
					"/" + g.name + "/state", rclcpp::QoS{10},
					[this, i](const gripper::msg::GripperState& m) {
						bool released = false;
						{
							const std::scoped_lock lock{this->mutex_};
							auto& g = this->grippers_[i];
							// 持っている把持機構の吸引が切れたら (手で置いたら)、持っていないことにする
							released = g.holding && g.state && g.state->suction_on && !m.suction_on;
							if (released) {
								g.holding = false;
							}
							g.state = m;
						}
						if (released) {
							RCLCPP_INFO(this->get_logger(), "%s: suction off; no longer holding a block", this->grippers_[i].name.c_str());
							this->publish_state();
						}
					},
					options
				);
			}

			// 棚の前: 中央の列の正面で、棚の方 (place.yaw) を向き、吸ったノーツの前の面 (一番前に出ている把持機構の
			// ノーツの、機体から遠い面) が棚の中心面から front_distance の所。棚は自チーム側 (+y) から -y 向きに使う
			{
				double reach = 0.0;
				for (const auto& g : this->grippers_) {
					reach = std::max(reach, g.offset_x + 0.5 * this->block_size_);
				}
				this->shelf_front_ = Pose2{
					.x = this->columns_x_[this->columns_x_.size() / 2],
					.y = this->shelf_center_y_ + front_distance + reach,
					.yaw = this->place_yaw_,
				};
			}

			// 2 個同時に回収するときの機体の姿勢: pick.pairs = [gripper0 のブロック, x, y, yaw, ...]
			// (gripper1 は gripper0 のブロックの 1 つ前 (-x 側) を取る)
			{
				const auto v = this->declare_parameter<std::vector<double>>("pick.pairs");
				if (v.size() % 4 != 0) {
					throw std::invalid_argument("pick.pairs must be [block, x, y, yaw, ...] (four numbers per pair)");
				}
				for (std::size_t i = 0; i < v.size(); i += 4) {
					this->pick_pairs_[static_cast<int>(v[i])] = Pose2{v[i + 1], v[i + 2], v[i + 3]};
				}
			}

			// 回収を始めるときの機体の姿勢: pick.poses.block<i> = [x, y, yaw] を把持機構の順に並べたもの
			for (std::size_t b = 0; b < this->blocks_x_.size(); ++b) {
				const auto name = std::format("pick.poses.block{}", b);
				const auto v = this->declare_parameter<std::vector<double>>(name);
				if (v.size() != 3 * this->grippers_.size()) {
					throw std::invalid_argument(name + " must be [x, y, yaw] for each gripper");
				}
				for (std::size_t i = 0; i < this->grippers_.size(); ++i) {
					// 赤では左右の把持機構の役が入れ替わる (青で gripper0 の列の姿勢は、赤では gripper1 の姿勢)
					const std::size_t column = this->red_ && this->grippers_.size() == 2 ? 1 - i : i;
					this->grippers_[i].pick_poses.push_back(Pose2{v[3 * column], v[3 * column + 1], v[3 * column + 2]});
				}
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
					this->odom_ = this->mirror(Pose2{
						m.pose.pose.position.x, m.pose.pose.position.y,
						std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
					});
					this->odom_stamp_ = this->now();
				},
				options
			);
			this->follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_action, this->group_);
			this->state_pub_ = this->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS{1}.transient_local());
			this->reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reset",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					if (this->busy_) {
						res->success = false;
						res->message = "a task is running";
						return;
					}
					{
						const std::scoped_lock lock{this->mutex_};
						std::ranges::fill(this->picked_, false);
						for (auto& g : this->grippers_) {
							g.holding = false;
						}
					}
					this->publish_state();
					res->success = true;
					res->message = "forgot picked blocks and held blocks";
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
			RCLCPP_INFO(this->get_logger(), "ready: %zu grippers, %zu blocks, %zu x %d shelf cells, shelf front (%.3f, %.3f, %.0f deg)",
				this->grippers_.size(), this->blocks_x_.size(), this->columns_x_.size(), this->rows_,
				this->shelf_front_.x, this->shelf_front_.y, this->shelf_front_.yaw * 180.0 / std::numbers::pi);
		}

	private:
		// ---------------------------------------------------------------- 受け付け

		auto check_goal(const RunTask::Goal& goal) const -> std::optional<std::string> {
			const auto gripper_ok = goal.gripper >= 0 && goal.gripper < std::ssize(this->grippers_);
			switch (goal.type) {
			case RunTask::Goal::PICK:
				if (!gripper_ok) return std::format("gripper {} does not exist", goal.gripper);
				if (goal.block < 0 || goal.block >= std::ssize(this->blocks_x_)) return std::format("block {} does not exist", goal.block);
				return std::nullopt;
			case RunTask::Goal::PICK_PAIR:
				if (this->grippers_.size() < 2) return std::string{"PICK_PAIR needs two grippers"};
				if (!this->pick_pairs_.contains(goal.block)) return std::format("no pick.pairs entry for block {}", goal.block);
				if (goal.block < 1 || goal.block >= std::ssize(this->blocks_x_)) return std::format("blocks {} and {} do not exist", goal.block, goal.block - 1);
				return std::nullopt;
			case RunTask::Goal::TO_SHELF_CELL:
				if (!gripper_ok) return std::format("gripper {} does not exist", goal.gripper);
				if (goal.column < 0 || goal.column >= std::ssize(this->columns_x_)) return std::format("column {} does not exist", goal.column);
				return std::nullopt;
			case RunTask::Goal::TO_SLALOM_ENTRANCE:
			case RunTask::Goal::TO_START:
			case RunTask::Goal::SLALOM_FORWARD:
			case RunTask::Goal::SLALOM_BACKWARD:
			case RunTask::Goal::TO_SLALOM_EXIT:
			case RunTask::Goal::TO_SHELF_FRONT:
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
			case RunTask::Goal::PICK_PAIR: status = this->pick_pair(goal.block); break;
			case RunTask::Goal::TO_SLALOM_ENTRANCE: status = this->go_to("slalom entrance", this->slalom_entrance_); break;
			case RunTask::Goal::TO_START: status = this->go_to("start", this->start_pose_); break;
			case RunTask::Goal::SLALOM_FORWARD: status = this->slalom(this->forward_route_); break;
			case RunTask::Goal::SLALOM_BACKWARD: status = this->slalom(this->backward_route_); break;
			case RunTask::Goal::TO_SHELF_CELL: status = this->to_shelf_cell(goal.gripper, goal.column); break;
			case RunTask::Goal::TO_SLALOM_EXIT: status = this->go_to("slalom exit", this->slalom_exit_); break;
			case RunTask::Goal::TO_SHELF_FRONT: status = this->go_to("shelf front", this->shelf_front_); break;
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
		/// 自チームの座標とフィールド座標の行き来 (赤なら y について反転する。反転は自分の逆)
		auto mirror(const Pose2& p) const -> Pose2 { return this->red_ ? Pose2{p.x, -p.y, -p.yaw} : p; }

		auto obstacles() -> Field {
			Field field{.bounds = this->field_.bounds, .obstacles = this->walls_};
			const std::scoped_lock lock{this->mutex_};
			const double h = 0.5 * this->block_size_;
			for (std::size_t i = 0; i < this->blocks_x_.size(); ++i) {
				if (!this->picked_[i]) {
					field.obstacles.push_back(Box{this->blocks_x_[i] - h, this->blocks_y_ - h, this->blocks_x_[i] + h, this->blocks_y_ + h});
				}
			}
			return field;
		}

		auto follow(const FollowPath::Goal& goal, const std::string& what) -> Status {
			if (!this->follow_client_->wait_for_action_server(1s)) return std::unexpected{"path_follower's follow action is not available"};
			auto goal_future = this->follow_client_->async_send_goal(goal);
			if (const auto s = this->wait([&] { return goal_future.wait_for(0s) == std::future_status::ready; }, 2.0, "sending " + what); !s) {
				return s;
			}
			const auto goal_handle = goal_future.get();
			if (!goal_handle) return std::unexpected{"path_follower rejected " + what};
			auto result_future = this->follow_client_->async_get_result(goal_handle);
			while (result_future.wait_for(20ms) != std::future_status::ready) {
				if (this->canceled() || !rclcpp::ok()) {
					this->follow_client_->async_cancel_goal(goal_handle);
					return std::unexpected{"canceled while " + what};
				}
			}
			const auto wrapped = result_future.get();
			if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result->success) {
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
			const auto worst = task_commander::sweep_clearance(this->obstacles(), sweep, this->footprint_, this->sweep_step_);
			if (worst.clearance < this->min_clearance_) {
				return std::unexpected{std::format("{}: the robot would come within {:.3f} m of an obstacle at ({:.3f}, {:.3f}) (min_clearance {:.3f}); not moving",
					what, worst.clearance, worst.x, this->red_ ? -worst.y : worst.y, this->min_clearance_)};
			}
			// 速さの違う区間は分けて走る
			const auto run = [&](const std::vector<Pose2>& part, const double speed) -> Status {
				if (part.empty()) return {};
				FollowPath::Goal goal{};
				for (const auto& q : part) {
					const auto p = this->mirror(q);   // 自チームの座標 -> フィールド座標
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
			auto& g = this->grippers_[static_cast<std::size_t>(gripper)];
			{
				const std::scoped_lock lock{this->mutex_};
				if (g.holding) return std::unexpected{g.name + " is already holding a block"};
				if (this->picked_[static_cast<std::size_t>(block)]) return std::unexpected{std::format("block {} was already picked (call ~/reset if it is back)", block)};
			}
			// 回収の姿勢は pick.poses で決める。機体から吸ったノーツの中心へ向かう向きの手前 (pick.standoff) から寄る
			const double reach = std::hypot(g.offset_x, g.offset_y);
			const auto along = [&](const Pose2& p, const double d) {
				return task_commander::offset_pose(p, d * g.offset_x / reach, d * g.offset_y / reach);
			};
			// 赤コートでは、さらに pick.red_extra_reach だけブロックへ寄る
			const auto target = along(g.pick_poses[static_cast<std::size_t>(block)], this->red_ ? this->pick_red_extra_reach_ : 0.0);
			const auto standoff = along(target, -this->pick_standoff_);

			this->step(std::format("{}: lift to clear, other lifts to carry", g.name));
			for (std::size_t i = 0; i < this->grippers_.size(); ++i) {
				const int index = static_cast<int>(i) == gripper ? this->lift_clear_ : this->lift_carry_;
				if (const auto s = this->set_lift(static_cast<int>(i), index); !s) return s;
			}
			// 吸引は寄る前に回しておく。吸う前に失敗したら、吸引を切ってから失敗を返す
			this->step(std::format("{}: suction on", g.name));
			if (const auto s = this->set_suction(gripper, true); !s) return s;
			const auto give_up = [&](const Status& s) -> Status {
				(void)this->set_suction(gripper, false);
				return s;
			};
			// 経由点 (pick.via) で先に回収の向きへ回ってから寄る。今の位置から直接寄ると、回りながら動く途中で
			// 後ろの LiDAR が壁 (スタートゾーンの境界の先端など) に当たることがある
			this->step(std::format("moving {} over block {}", g.name, block));
			if (const auto s = this->move({this->pick_via_, standoff, target}, std::format("moving {} over block {}", g.name, block), this->approach_speed_); !s) return give_up(s);
			this->step(std::format("{}: down", g.name));
			if (const auto s = this->set_lift(gripper, this->lift_pick_); !s) return give_up(s);
			if (const auto s = this->sleep(this->grip_time_, "gripping"); !s) return give_up(s);
			{
				const std::scoped_lock lock{this->mutex_};
				g.holding = true;
				this->picked_[static_cast<std::size_t>(block)] = true;
			}
			this->publish_state();
			this->step(std::format("{}: up to carry", g.name));
			if (const auto s = this->set_lift(gripper, this->lift_carry_); !s) return s;
			this->step("backing off");
			return this->move({standoff}, "backing off from the notes", this->approach_speed_);
		}

		/// block と block - 1 を同時に回収する (青なら gripper0 が block、赤なら gripper1 が block)。手順は pick と同じで、昇降と吸引は 2 つ同時に動かす
		auto pick_pair(const int block) -> Status {
			// 青では gripper0 (左) が block、gripper1 (右) が block - 1 を取る。赤では左右の役が入れ替わる
			const std::array<int, 2> grippers = this->red_ ? std::array<int, 2>{1, 0} : std::array<int, 2>{0, 1};
			const std::array<int, 2> blocks{block, block - 1};
			{
				const std::scoped_lock lock{this->mutex_};
				for (std::size_t i = 0; i < 2; ++i) {
					const auto& g = this->grippers_[static_cast<std::size_t>(grippers[i])];
					if (g.holding) return std::unexpected{g.name + " is already holding a block"};
					if (this->picked_[static_cast<std::size_t>(blocks[i])]) return std::unexpected{std::format("block {} was already picked (call ~/reset if it is back)", blocks[i])};
				}
			}
			// 赤コートでは、さらに pick.red_extra_reach だけ前 (ブロックの方) へ寄る
			const auto target = task_commander::offset_pose(this->pick_pairs_.at(block), this->red_ ? this->pick_red_extra_reach_ : 0.0, 0.0);
			// 手前は機体の後ろ向きに pick.standoff (2 つの把持機構は機体の前に左右ほぼ対称に並ぶ)
			const auto standoff = task_commander::offset_pose(target, -this->pick_standoff_, 0.0);
			const auto lifts = [&](const int index) -> Status {
				for (const int g : grippers) {
					if (const auto s = this->set_lift(g, index, false); !s) return s;
				}
				for (const int g : grippers) {
					if (const auto s = this->set_lift(g, index, true); !s) return s;
				}
				return {};
			};
			const auto suction = [&](const bool on) -> Status {
				for (const int g : grippers) {
					if (const auto s = this->set_suction(g, on); !s) return s;
				}
				return {};
			};
			const auto what = std::format("blocks {} and {}", blocks[0], blocks[1]);

			this->step("lifts to clear");
			if (const auto s = lifts(this->lift_clear_); !s) return s;
			// 吸引は寄る前に回しておく。吸う前に失敗したら、吸引を切ってから失敗を返す
			this->step("suction on");
			if (const auto s = suction(true); !s) return s;
			const auto give_up = [&](const Status& s) -> Status {
				(void)suction(false);
				return s;
			};
			this->step("moving over " + what);
			if (const auto s = this->move({this->pick_via_, standoff, target}, "moving over " + what, this->approach_speed_); !s) return give_up(s);
			this->step("lifts down");
			if (const auto s = lifts(this->lift_pick_); !s) return give_up(s);
			if (const auto s = this->sleep(this->grip_time_, "gripping"); !s) return give_up(s);
			{
				const std::scoped_lock lock{this->mutex_};
				for (std::size_t i = 0; i < 2; ++i) {
					this->grippers_[static_cast<std::size_t>(grippers[i])].holding = true;
					this->picked_[static_cast<std::size_t>(blocks[i])] = true;
				}
			}
			this->publish_state();
			this->step("lifts up to carry");
			if (const auto s = lifts(this->lift_carry_); !s) return s;
			this->step("backing off");
			return this->move({standoff}, "backing off from the notes", this->approach_speed_);
		}

		/// 今の y のまま、把持機構 gripper の吸ったノーツの x を列 column の中心に合わせ、棚の方 (place.yaw) を向く。
		/// 前後 (棚へ差し込む) と昇降・吸引は変えない (手でする)
		auto to_shelf_cell(const int gripper, const int column) -> Status {
			const auto here = this->odom();
			if (!here) return std::unexpected{"no recent pose estimate"};
			const auto& g = this->grippers_[static_cast<std::size_t>(gripper)];
			// 向きが決まれば道具の x は機体の y によらないので、y だけ今の値に置き換える
			auto target = task_commander::pose_for_tool(
				this->columns_x_[static_cast<std::size_t>(column)], here->y, this->place_yaw_, g.offset_x, g.offset_y);
			target.y = here->y;
			const auto what = std::format("lining {} up with column {}", g.name, column);
			this->step(what);
			return this->move({target}, what, this->approach_speed_);
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
		task_commander::Footprint footprint_{};
		double min_clearance_{};
		Pose2 start_pose_{};
		Pose2 slalom_entrance_{};
		Pose2 slalom_exit_{};
		std::string forward_route_{};
		std::string backward_route_{};
		std::vector<double> blocks_x_{};
		double blocks_y_{};
		double block_size_{};
		bool red_{false};
		Pose2 pick_via_{};
		std::map<int, Pose2> pick_pairs_{};
		double pick_standoff_{};
		double pick_red_extra_reach_{};
		double grip_time_{};
		std::vector<double> columns_x_{};
		int rows_{};
		double place_yaw_{};
		double shelf_center_y_{};
		Pose2 shelf_front_{};
		int lift_pick_{};
		int lift_clear_{};
		int lift_carry_{};
		double lift_timeout_{};
		double travel_speed_{};
		double approach_speed_{};
		double sweep_step_{};
		std::vector<Gripper> grippers_{};

		// 覚えていること (mutex_)
		std::vector<bool> picked_{};
		std::optional<Pose2> odom_{};
		rclcpp::Time odom_stamp_{};

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
