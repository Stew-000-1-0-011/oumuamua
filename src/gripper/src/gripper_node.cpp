/// 把持機構 (昇降 + 吸引) 1 つぶんを動かすノード。機構の数だけ別の名前で立てる (gripper0, gripper1, ...)。
///
/// 昇降: ロボマス (C610 / C620) を、robomas_plugins の USB-CAN ボードの位置制御で動かす。
/// - 起動時に位置制御の設定 (robomas_frame) を送り、以後 control_rate で位置の目標 (robomas_target<k>) を送る
/// - 位置の目標は、ゴールへ向けて lift.max_speed で動かしながら送る (急に飛ばさない)
/// - 使う高さ (下・中段・上段など) は lift.positions に並べておき、~/set_lift_position で番号を送って選ぶ
/// - 高さは起動時の位置を 0 とする。ボードへの目標は lift.target_scale * 高さ + lift.target_offset
/// - ロボマスのフィードバック (CAN 0x200 + k) が robomas_can_rx に届いていれば、ロータの回転を数え上げて
///   今の高さとする。届いていなければ、送っている指令を今の高さとみなす
///
/// 吸引: mini-shirasu を動かす。制御は mini_shirasu_node が行う (mode: current なら電流制御、duty ならデューティ)。
/// オンなら suction.target、オフなら 0 を、control_rate で suction.target_topic (その mode の ~/target_*) に送り続ける
/// (mini_shirasu_node は目標が途切れると 0 A にする)。
///
/// | 方向 | 名前 | 型 |
/// | --- | --- | --- |
/// | srv | `~/set_lift_position` | gripper/srv/SetLiftPosition (lift.positions の何番目の高さへ動かすか) |
/// | action | `~/move_lift` | gripper/action/MoveLift (高さ [m] を直接指定。調整用) |
/// | srv | `~/set_suction` | std_srvs/SetBool (オン/オフ) |
/// | srv | `~/enable_lift` | std_srvs/SetBool (true で位置制御、false で無効) |
/// | srv | `~/zero_lift` | std_srvs/Trigger (今の位置を高さ 0 にし直す) |
/// | pub | `~/state` | gripper/msg/GripperState |

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robomas_plugins/msg/frame.hpp>
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gripper/action/move_lift.hpp"
#include "gripper/lift.hpp"
#include "gripper/current_guard.hpp"
#include "gripper/msg/gripper_state.hpp"
#include "gripper/srv/set_lift_position.hpp"

namespace {
	using MoveLift = gripper::action::MoveLift;
	using GoalHandle = rclcpp_action::ServerGoalHandle<MoveLift>;
	using SetLiftPosition = gripper::srv::SetLiftPosition;

	/// 昇降の動きの、1 周期ごとの結果
	enum class Motion { idle, moving, arrived, timed_out };

	/// robomas_plugins のボードの制御モード (RobomasFrame.mode)
	constexpr std::uint8_t mode_disable = 0;
	constexpr std::uint8_t mode_position = 2;

	class GripperNode final : public rclcpp::Node {
	public:
		GripperNode() : rclcpp::Node("gripper") {
			const double rate = this->declare_parameter<double>("control_rate");
			this->dt_ = 1.0 / rate;

			// --- 昇降 ---
			this->motor_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.motor"));
			if (this->motor_ < 1 || this->motor_ > 8) {
				throw std::invalid_argument("lift.motor must be 1..8");
			}
			this->c620_ = this->declare_parameter<bool>("lift.c620");
			this->vel_kp_ = this->declare_parameter<double>("lift.vel_kp");
			this->vel_ki_ = this->declare_parameter<double>("lift.vel_ki");
			this->pos_kp_ = this->declare_parameter<double>("lift.pos_kp");
			this->temperature_limit_ = static_cast<std::uint8_t>(this->declare_parameter<std::int64_t>("lift.temperature_limit"));
			this->target_scale_ = this->declare_parameter<double>("lift.target_scale");
			this->target_offset_ = this->declare_parameter<double>("lift.target_offset");
			this->gear_ratio_ = this->declare_parameter<double>("lift.gear_ratio");
			this->meters_per_output_revolution_ = this->declare_parameter<double>("lift.meters_per_output_revolution");
			this->min_height_ = this->declare_parameter<double>("lift.min_height");
			this->max_height_ = this->declare_parameter<double>("lift.max_height");
			this->max_speed_ = this->declare_parameter<double>("lift.max_speed");
			this->tolerance_ = this->declare_parameter<double>("lift.tolerance");
			this->settle_time_ = this->declare_parameter<double>("lift.settle_time");
			this->timeout_margin_ = this->declare_parameter<double>("lift.timeout_margin");
			this->feedback_timeout_ = this->declare_parameter<double>("lift.feedback_timeout");
			this->current_guard_.emplace(
				this->declare_parameter<double>("lift.overcurrent_threshold"),
				this->declare_parameter<double>("lift.overcurrent_duration"), this->feedback_timeout_);
			this->overcurrent_backoff_ = this->declare_parameter<double>("lift.overcurrent_backoff");
			if (!std::isfinite(this->overcurrent_backoff_) || this->overcurrent_backoff_ < 0.0) {
				throw std::invalid_argument("lift.overcurrent_backoff must be finite and not negative");
			}
			if (!std::isfinite(this->target_scale_) || this->target_scale_ == 0.0
				|| !std::isfinite(this->gear_ratio_) || this->gear_ratio_ <= 0.0
				|| !std::isfinite(this->meters_per_output_revolution_) || this->meters_per_output_revolution_ <= 0.0) {
				throw std::invalid_argument("lift current protection requires a valid target scale, gear ratio and height feedback scale");
			}
			this->config_repeat_ = static_cast<int>(this->declare_parameter<std::int64_t>("lift.config_repeat"));
			this->positions_ = this->declare_parameter<std::vector<double>>("lift.positions");
			this->position_names_ = this->declare_parameter<std::vector<std::string>>("lift.position_names");
			if (!this->position_names_.empty() && this->position_names_.size() != this->positions_.size()) {
				throw std::invalid_argument("lift.position_names must be empty or as long as lift.positions");
			}
			for (const double h : this->positions_) {
				if (!std::isfinite(h) || h < this->min_height_ || h > this->max_height_) {
					throw std::invalid_argument("lift.positions must be within lift.min_height..lift.max_height");
				}
			}
			const bool enable_on_start = this->declare_parameter<bool>("lift.enable_on_start");

			// --- 吸引 ---
			this->suction_target_ = this->declare_parameter<double>("suction.target");
			const auto suction_topic = this->declare_parameter<std::string>("suction.target_topic");
			if (suction_topic.empty()) {
				throw std::invalid_argument("suction.target_topic must be set (mini_shirasu_node's ~/target_current or ~/target_duty)");
			}

			// --- 通信 ---
			const auto frame_topic = this->declare_parameter<std::string>("robomas_frame_topic");
			auto target_topic = this->declare_parameter<std::string>("robomas_target_topic");
			if (target_topic.empty()) {
				target_topic = "/robomas_target" + std::to_string(this->motor_);
			}
			const auto can_rx_topic = this->declare_parameter<std::string>("can_rx_topic");

			this->frame_pub_ = this->create_publisher<robomas_plugins::msg::RobomasFrame>(frame_topic, rclcpp::QoS{10});
			this->target_pub_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>(target_topic, rclcpp::QoS{10});
			this->suction_pub_ = this->create_publisher<std_msgs::msg::Float64>(suction_topic, rclcpp::QoS{10});
			this->state_pub_ = this->create_publisher<gripper::msg::GripperState>("~/state", rclcpp::QoS{10});
			this->can_rx_sub_ = this->create_subscription<robomas_plugins::msg::Frame>(
				can_rx_topic, rclcpp::QoS{100}, [this](const robomas_plugins::msg::Frame& m) { this->on_can_rx(m); }
			);

			this->suction_srv_ = this->create_service<std_srvs::srv::SetBool>(
				"~/set_suction",
				[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
					this->suction_on_ = req->data;
					res->success = true;
					res->message = req->data ? "suction on" : "suction off";
					RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
				}
			);
			this->position_srv_ = this->create_service<SetLiftPosition>(
				"~/set_lift_position",
				[this](const std::shared_ptr<SetLiftPosition::Request> req, std::shared_ptr<SetLiftPosition::Response> res) {
					this->on_set_position(*req, *res);
				}
			);
			this->enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
				"~/enable_lift",
				[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
					res->success = this->set_lift_enabled(req->data);
					res->message = !res->success ? "fresh motor feedback required before reinitializing position control"
						: (req->data ? "lift enabled (position control)" : "lift disabled");
				}
			);
			this->zero_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/zero_lift",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					if (!this->feedback_valid()) {
						res->success = false;
						res->message = "fresh motor feedback required before zeroing";
						return;
					}
					this->zero_lift();
					res->success = true;
					res->message = "lift zeroed at the current position";
				}
			);
			this->action_server_ = rclcpp_action::create_server<MoveLift>(
				this,
				"~/move_lift",
				[this](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveLift::Goal> goal) {
					return this->on_goal(*goal);
				},
				[this](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
				[this](const std::shared_ptr<GoalHandle> handle) { this->on_accepted(handle); }
			);

			if (enable_on_start) {
				this->set_lift_enabled(true);
			}
			this->timer_ = this->create_wall_timer(std::chrono::duration<double>(this->dt_), [this] { this->on_timer(); });
			this->config_timer_ = this->create_wall_timer(std::chrono::seconds{1}, [this] { this->resend_config(); });
			RCLCPP_INFO(this->get_logger(), "lift: motor %d (%s), height %.3f..%.3f m; suction: %.3f -> %s",
				this->motor_, this->c620_ ? "C620" : "C610", this->min_height_, this->max_height_,
				this->suction_target_, suction_topic.c_str());
		}

		~GripperNode() override {
			// 落ちるときは昇降を無効にし、吸引を止める (届くとは限らない)
			if (this->lift_enabled_) {
				this->send_mode(mode_disable);
			}
			std_msgs::msg::Float64 zero{};
			this->suction_pub_->publish(zero);
		}

	private:
		// ---------------------------------------------------------------- 昇降

		auto set_lift_enabled(const bool enabled) -> bool {
			if (enabled && this->position_initialized_ && this->counter_.valid() && !this->feedback_valid()) {
				return false;
			}
			this->current_guard_->reset_timer();
			this->lift_enabled_ = enabled;
			this->config_sent_ = 0;
			this->send_mode(enabled ? mode_position : mode_disable);
			if (!enabled) {
				this->abort_goal("lift disabled");
				this->stop_move();
			}
			RCLCPP_INFO(this->get_logger(), "lift %s", enabled ? "enabled (position control)" : "disabled");
			return true;
		}

		/// 今の位置を高さ 0 にし直す (起動したときと同じ状態にする)。
		/// ボードには原点を決める命令が無く、位置制御を有効にした位置を 0 とするので、無効 -> 有効を送り直す。
		/// 無効のときは、次に有効にした位置が 0 になる
		void zero_lift() {
			this->current_guard_->shift_origin(this->measured_height());
			this->abort_goal("lift zeroed");
			this->stop_move();
			this->ramp_ = gripper::Ramp{0.0};
			this->counter_.reset();
			this->last_feedback_.reset();
			this->board_zero_target_ = 0.0;
			this->position_initialized_ = false;
			if (this->lift_enabled_) {
				this->send_mode(mode_disable);
				this->send_mode(mode_position);
			}
			RCLCPP_INFO(this->get_logger(), "lift zeroed at the current position");
		}

		void send_mode(const std::uint8_t mode) {
			// After initial setup a reset needs fresh position feedback; otherwise
			// the new firmware origin cannot be related to our height coordinates.
			if (mode == mode_position && this->position_initialized_ && this->counter_.valid() && !this->feedback_valid()) {
				return;
			}
			// Position mode resets both the firmware integrator and its rotor origin.
			// Keep our physical height coordinates across that reset.
			if (mode == mode_position && this->feedback_valid()) {
				this->board_zero_target_ = this->target_scale_ * this->measured_height() + this->target_offset_;
			}
			robomas_plugins::msg::RobomasFrame frame{};
			frame.header.stamp = this->now();
			frame.motor = static_cast<std::uint8_t>(this->motor_ - 1);
			frame.c620 = this->c620_;
			frame.mode = mode;
			frame.temp = this->temperature_limit_;
			frame.velkp = static_cast<float>(this->vel_kp_);
			frame.velki = static_cast<float>(this->vel_ki_);
			frame.poskp = static_cast<float>(this->pos_kp_);
			this->frame_pub_->publish(frame);
			if (mode == mode_position) {
				this->position_initialized_ = true;
			}
			++this->config_sent_;
		}

		/// ブリッジがまだつながっていないと設定が捨てられるので、起動直後に何度か送り直す
		void resend_config() {
			if (this->config_sent_ < this->config_repeat_) {
				this->send_mode(this->lift_enabled_ ? mode_position : mode_disable);
			}
		}

		void on_can_rx(const robomas_plugins::msg::Frame& m) {
			if (m.is_rtr || m.is_extended || m.is_error) {
				return;
			}
			const std::array<std::uint8_t, 8> data{m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5], m.data[6], m.data[7]};
			const auto fb = gripper::parse_feedback(m.id, m.dlc, data, this->motor_);
			if (!fb) {
				return;
			}
			this->counter_.update(fb->angle);
			const auto now = gripper::CurrentGuard::Clock::now();
			this->last_feedback_ = now;
			this->lift_current_ = gripper::current_amps(fb->current, this->c620_);
			const double actual = this->measured_height();
			// Use height direction (not raw rotor/current polarity: the two lifts are mirrored).
			const double error = this->ramp_.position() - actual;
			const int direction = this->lift_enabled_ ? ((error > 1e-6) - (error < -1e-6)) : 0;
			const double effort = this->lift_enabled_ ? this->lift_current_ * (this->target_scale_ < 0.0 ? -1.0 : 1.0) : 0.0;
			const int stopped = this->current_guard_->sample(effort, direction, actual, now);
			if (stopped != 0) {
				this->abort_goal("lift overcurrent: travel limit detected");
				this->stop_move();
				this->ramp_ = gripper::Ramp{actual};
				// Holding the old command would continue pushing into the end stop.
				// Reinitialize position control to remove stored PI effort, hold here,
				// then step back from the stop by lift.overcurrent_backoff (the timer ramps there).
				this->send_mode(mode_position);
				this->config_sent_ = this->config_repeat_;
				this->publish_lift_target(actual);
				const double backoff = std::clamp(actual - stopped * this->overcurrent_backoff_, this->min_height_, this->max_height_);
				this->ramp_.set_goal(backoff);
				RCLCPP_ERROR(this->get_logger(), "lift overcurrent %.2f A: blocked %s at %.4f m; backing off to %.4f m",
					this->lift_current_, stopped > 0 ? "upward" : "downward", actual, backoff);
			}
		}

		auto feedback_valid() -> bool {
			return this->counter_.valid() && this->meters_per_output_revolution_ != 0.0 && this->last_feedback_
				&& std::chrono::duration<double>(gripper::CurrentGuard::Clock::now() - *this->last_feedback_).count()
					<= this->feedback_timeout_;
		}

		auto measured_height() const -> double {
			const double sign = this->target_scale_ < 0.0 ? -1.0 : 1.0;
			return sign * this->counter_.revolutions() / this->gear_ratio_ * this->meters_per_output_revolution_;
		}

		void publish_lift_target(const double height) {
			robomas_plugins::msg::RobomasTarget target{};
			target.header.stamp = this->now();
			target.target = static_cast<float>(this->target_scale_ * height + this->target_offset_ - this->board_zero_target_);
			this->target_pub_->publish(target);
		}

		/// 今の高さ [m]。フィードバックが無ければ送っている指令
		auto height() -> double {
			if (this->feedback_valid()) {
				// ロータは目標と同じ向きに数える: target_scale が負なら、高さが増えるとロータの回転は減る
				return this->measured_height();
			}
			return this->ramp_.position();
		}

		// ---------------------------------------------------------------- 動かし方

		/// goal [m] へ動き出す。index は lift.positions の番号 (高さを直接指定したなら -1)
		void start_move(const double goal, const int index) {
			this->ramp_.set_goal(goal);
			this->position_index_ = index;
			this->moving_ = true;
			this->arrived_ = false;
			this->settled_since_.reset();
			const double distance = std::abs(goal - this->ramp_.position());
			this->deadline_ = this->now() + rclcpp::Duration::from_seconds(distance / std::max(this->max_speed_, 1e-6) + this->timeout_margin_);
		}

		/// 今の指令で止める
		void stop_move() {
			this->ramp_.hold();
			this->position_index_ = -1;
			this->moving_ = false;
			this->arrived_ = false;
			this->settled_since_.reset();
		}

		/// 指令がゴールに着き、(フィードバックがあれば) 実際の高さも tolerance 以内に settle_time 留まったら着いた
		auto update_motion() -> Motion {
			if (!this->moving_) {
				return Motion::idle;
			}
			const auto now = this->now();
			const bool close = this->ramp_.arrived() && !this->current_guard_->pending()
				&& (!this->feedback_valid() || std::abs(this->height() - this->ramp_.goal()) <= this->tolerance_);
			if (close) {
				if (!this->settled_since_) {
					this->settled_since_ = now;
				}
				if ((now - *this->settled_since_).seconds() >= this->settle_time_) {
					this->moving_ = false;
					this->arrived_ = true;
					return Motion::arrived;
				}
			} else {
				this->settled_since_.reset();
			}
			if (now > this->deadline_) {
				this->moving_ = false;
				RCLCPP_WARN(this->get_logger(), "lift timed out at %.3f m (goal %.3f m)", this->height(), this->ramp_.goal());
				return Motion::timed_out;
			}
			return Motion::moving;
		}

		auto position_label(const int index) const -> std::string {
			if (index >= 0 && static_cast<std::size_t>(index) < this->position_names_.size()) {
				return std::to_string(index) + " (" + this->position_names_[static_cast<std::size_t>(index)] + ")";
			}
			return std::to_string(index);
		}

		// ---------------------------------------------------------------- サービス

		void on_set_position(const SetLiftPosition::Request& req, SetLiftPosition::Response& res) {
			if (!this->lift_enabled_) {
				res.success = false;
				res.message = "lift is disabled";
			} else if (req.index < 0 || static_cast<std::size_t>(req.index) >= this->positions_.size()) {
				res.success = false;
				res.message = "index " + std::to_string(req.index) + " is out of lift.positions (size "
					+ std::to_string(this->positions_.size()) + ")";
			} else {
				const double goal = this->positions_[static_cast<std::size_t>(req.index)];
				if (!this->feedback_valid() || !this->current_guard_->permits(goal)) {
					res.success = false;
					res.message = !this->feedback_valid() ? "motor feedback is unavailable" : "goal is beyond an overcurrent travel limit";
					return;
				}
				this->abort_goal("preempted by set_lift_position");
				this->start_move(goal, req.index);
				res.success = true;
				res.message = "moving to position " + this->position_label(req.index) + ": " + std::to_string(goal) + " m";
			}
			if (res.success) {
				RCLCPP_INFO(this->get_logger(), "%s", res.message.c_str());
			} else {
				RCLCPP_WARN(this->get_logger(), "set_lift_position rejected: %s", res.message.c_str());
			}
		}

		// ---------------------------------------------------------------- アクション

		auto on_goal(const MoveLift::Goal& goal) -> rclcpp_action::GoalResponse {
			if (!this->lift_enabled_) {
				RCLCPP_WARN(this->get_logger(), "move_lift rejected: lift is disabled");
				return rclcpp_action::GoalResponse::REJECT;
			}
			if (!std::isfinite(goal.height)) {
				return rclcpp_action::GoalResponse::REJECT;
			}
			if (!this->feedback_valid()
				|| !this->current_guard_->permits(std::clamp(goal.height, this->min_height_, this->max_height_))) {
				return rclcpp_action::GoalResponse::REJECT;
			}
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		void on_accepted(const std::shared_ptr<GoalHandle> handle) {
			const double requested = handle->get_goal()->height;
			const double goal = std::clamp(requested, this->min_height_, this->max_height_);
			// A CAN callback may have tripped the guard since on_goal accepted this request.
			if (!this->lift_enabled_ || !this->feedback_valid() || !this->current_guard_->permits(goal)) {
				auto result = std::make_shared<MoveLift::Result>();
				result->reached = false;
				result->height = this->height();
				result->message = "lift unavailable or goal blocked by overcurrent travel limit";
				handle->abort(result);
				return;
			}
			this->abort_goal("preempted by a new goal");
			if (goal != requested) {
				RCLCPP_WARN(this->get_logger(), "move_lift: %.3f m is out of range; going to %.3f m", requested, goal);
			}
			this->start_move(goal, -1);
			this->goal_handle_ = handle;
		}

		void abort_goal(const char* why) {
			if (this->goal_handle_ && this->goal_handle_->is_active()) {
				auto result = std::make_shared<MoveLift::Result>();
				result->reached = false;
				result->height = this->height();
				result->message = why;
				this->goal_handle_->abort(result);
			}
			this->goal_handle_.reset();
		}

		/// アクションのゴールがあれば、取り消し・途中経過・結果を返す
		void update_goal(const Motion motion) {
			if (!this->goal_handle_) {
				return;
			}
			auto result = std::make_shared<MoveLift::Result>();
			result->height = this->height();
			switch (motion) {
			case Motion::arrived:
				result->reached = true;
				result->message = this->feedback_valid() ? "reached" : "reached (by the command; no motor feedback)";
				this->goal_handle_->succeed(result);
				this->goal_handle_.reset();
				return;
			case Motion::timed_out:
			case Motion::idle:
				result->reached = false;
				result->message = motion == Motion::timed_out ? "timed out" : "stopped";
				this->goal_handle_->abort(result);
				this->goal_handle_.reset();
				return;
			case Motion::moving:
				break;
			}
			auto feedback = std::make_shared<MoveLift::Feedback>();
			feedback->height = result->height;
			feedback->target = this->ramp_.position();
			this->goal_handle_->publish_feedback(feedback);
		}

		void cancel_goal_if_requested() {
			if (this->goal_handle_ && this->goal_handle_->is_canceling()) {
				this->stop_move();
				auto result = std::make_shared<MoveLift::Result>();
				result->height = this->height();
				result->reached = false;
				result->message = "canceled";
				this->goal_handle_->canceled(result);
				this->goal_handle_.reset();
			}
		}

		// ---------------------------------------------------------------- 周期処理

		void on_timer() {
			if (this->lift_enabled_) {
				const double command = this->ramp_.step(this->dt_, this->max_speed_);
				this->publish_lift_target(command);
			}
			this->cancel_goal_if_requested();
			this->update_goal(this->update_motion());

			std_msgs::msg::Float64 current{};
			current.data = this->suction_on_ ? this->suction_target_ : 0.0;
			this->suction_pub_->publish(current);

			gripper::msg::GripperState state{};
			state.header.stamp = this->now();
			state.lift_height = this->height();
			state.lift_command = this->ramp_.position();
			state.lift_goal = this->ramp_.goal();
			state.lift_position = this->position_index_;
			state.lift_moving = this->moving_;
			state.lift_arrived = this->arrived_;
			state.lift_feedback = this->feedback_valid();
			state.lift_current = this->lift_current_;
			state.lift_upper_limited = this->current_guard_->upper().has_value();
			state.lift_lower_limited = this->current_guard_->lower().has_value();
			state.lift_upper_limit = this->current_guard_->upper().value_or(this->max_height_);
			state.lift_lower_limit = this->current_guard_->lower().value_or(this->min_height_);
			state.suction_on = this->suction_on_;
			state.suction_target = current.data;
			this->state_pub_->publish(state);
		}

		// 設定
		double dt_{};
		int motor_{};
		bool c620_{};
		double vel_kp_{};
		double vel_ki_{};
		double pos_kp_{};
		std::uint8_t temperature_limit_{};
		double target_scale_{};
		double target_offset_{};
		double gear_ratio_{};
		double meters_per_output_revolution_{};
		double min_height_{};
		double max_height_{};
		double max_speed_{};
		double tolerance_{};
		double settle_time_{};
		double timeout_margin_{};
		double feedback_timeout_{};
		int config_repeat_{};
		double suction_target_{};
		std::vector<double> positions_{};
		std::vector<std::string> position_names_{};

		// 状態
		bool lift_enabled_{false};
		int config_sent_{0};
		gripper::Ramp ramp_{0.0};
		gripper::RotorCounter counter_{};
		std::optional<gripper::CurrentGuard::TimePoint> last_feedback_{};
		std::optional<gripper::CurrentGuard> current_guard_{};
		double overcurrent_backoff_{};
		double lift_current_{};
		double board_zero_target_{};
		bool position_initialized_{false};
		std::shared_ptr<GoalHandle> goal_handle_{};
		bool moving_{false};
		bool arrived_{false};
		int position_index_{-1};
		std::optional<rclcpp::Time> settled_since_{};
		rclcpp::Time deadline_{};
		bool suction_on_{false};

		rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr frame_pub_{};
		rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr target_pub_{};
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr suction_pub_{};
		rclcpp::Publisher<gripper::msg::GripperState>::SharedPtr state_pub_{};
		rclcpp::Subscription<robomas_plugins::msg::Frame>::SharedPtr can_rx_sub_{};
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr suction_srv_{};
		rclcpp::Service<SetLiftPosition>::SharedPtr position_srv_{};
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_{};
		rclcpp_action::Server<MoveLift>::SharedPtr action_server_{};
		rclcpp::TimerBase::SharedPtr timer_{};
		rclcpp::TimerBase::SharedPtr config_timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<GripperNode>());
	} catch (const std::exception& e) {
		// パラメータが足りないときなど (既定値は持たないので、全部パラメータファイルで与える)
		RCLCPP_FATAL(rclcpp::get_logger("gripper"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
