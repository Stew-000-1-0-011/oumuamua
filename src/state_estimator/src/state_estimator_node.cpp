/// @file state_estimator_node.cpp
/// 機体の状態 (平面の姿勢と機体速度) を推定するノード。sotoba_node と対になって動く。
///
/// - 予測: `~/body_velocity` (下位層が推定した機体速度、機体座標系) で姿勢を進める
/// - 観測: sotoba_node の事後分布 (`posterior_beliefs`) のフィールド物体から機体の姿勢を求めて入れる。
///         事後には自分が送った事前が含まれているので、その分を差し引いてから入れる (二重計上を避ける)
/// - 出力: 現在の推定を `~/odom` と TF (field -> base_link) に、
///         sotoba_node への事前分布を `prior_beliefs` に、制御周期程度で出す
///
/// 循環 (事前 -> sotoba -> 事後 -> ここ) の中で時刻をまたいで状態を持つのはこのノードだけにする。
/// リセット (`~/reset`、`/initialpose`) はここで行い、リセット前の事前から作られた事後は時刻で捨てる。

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sotoba_ros/msg/belief_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "state_estimator/belief_geometry.hpp"
#include "state_estimator/filter.hpp"

namespace {
	using sotoba_ros::msg::BeliefArray;
	using state_estimator::Belief;
	using state_estimator::Filter;
	using state_estimator::FilterParams;
	using state_estimator::Mat3;
	using state_estimator::Mat6;
	using state_estimator::SE3;
	using state_estimator::UpdateResult;
	using state_estimator::Vec3;

	/// sotoba の status (BeliefArray.status)
	constexpr std::uint8_t status_updated = 1;

	auto to_se3(const geometry_msgs::msg::Pose& p) -> SE3 {
		const Eigen::Quaterniond q{p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
		return SE3{Sophus::SO3d{q.normalized()}, Eigen::Vector3d{p.position.x, p.position.y, p.position.z}};
	}

	auto to_pose(const SE3& T) -> geometry_msgs::msg::Pose {
		geometry_msgs::msg::Pose p{};
		p.position.x = T.translation().x();
		p.position.y = T.translation().y();
		p.position.z = T.translation().z();
		const auto q = T.unit_quaternion();
		p.orientation.x = q.x();
		p.orientation.y = q.y();
		p.orientation.z = q.z();
		p.orientation.w = q.w();
		return p;
	}

	auto yaw_quaternion(const double yaw) -> geometry_msgs::msg::Quaternion {
		geometry_msgs::msg::Quaternion q{};
		q.z = std::sin(0.5 * yaw);
		q.w = std::cos(0.5 * yaw);
		return q;
	}

	class StateEstimatorNode final : public rclcpp::Node {
	public:
		StateEstimatorNode() : rclcpp::Node("state_estimator") {
			// --- 起動時のみ ---
			this->field_frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
			this->lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "laser");
			this->field_object_ = this->declare_parameter<std::string>("field_object", "field");
			const auto posterior_topic =
				this->declare_parameter<std::string>("posterior_topic", "/sotoba_node/posterior_beliefs");
			const auto prior_topic = this->declare_parameter<std::string>("prior_topic", "/sotoba_node/prior_beliefs");
			const auto initial_topic =
				this->declare_parameter<std::string>("initial_topic", "/sotoba_node/initial_beliefs");
			this->publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
			const double rate = this->declare_parameter<double>("publish_rate", 100.0);

			// --- 実行中に変えられる ---
			this->declare_parameter<double>("input_delay", 0.0);
			this->declare_parameter<double>("measurement_timeout", 0.5);
			this->declare_parameter<double>("initial_sigma_xy", 0.05);
			this->declare_parameter<double>("initial_sigma_yaw", 0.05);
			this->declare_parameter<bool>("subtract_prior", true);
			this->declare_parameter<double>("measurement_floor_xy", 0.002);
			this->declare_parameter<double>("measurement_floor_yaw", 0.002);
			this->declare_parameter<int>("inflate_after_failures", 3);
			this->declare_parameter<double>("inflate_sigma_xy", 0.05);
			this->declare_parameter<double>("inflate_sigma_yaw", 0.05);
			this->declare_parameter<bool>("track_relations", true);

			this->declare_parameter<double>("filter.tau_linear", 0.0);
			this->declare_parameter<double>("filter.tau_angular", 0.0);
			this->declare_parameter<double>("filter.velocity_noise_linear", 1.0);
			this->declare_parameter<double>("filter.velocity_noise_angular", 4.0);
			this->declare_parameter<double>("filter.input_sigma_linear", 0.05);
			this->declare_parameter<double>("filter.input_sigma_angular", 0.1);
			this->declare_parameter<double>("filter.pose_noise_linear", 0.01);
			this->declare_parameter<double>("filter.pose_noise_angular", 0.01);
			this->declare_parameter<double>("filter.gate_sigma", 0.0);
			this->declare_parameter<double>("filter.max_horizon", 0.5);

			this->declare_parameter<double>("prior.sigma_z", 0.005);
			this->declare_parameter<double>("prior.sigma_tilt", 0.005);
			this->declare_parameter<double>("prior.relation_sigma_xy", 0.02);
			this->declare_parameter<double>("prior.relation_sigma_yaw", 0.05);

			this->load_params();
			this->param_cb_ = this->add_post_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter>&) { this->load_params(); }
			);

			// --- 入力 ---
			this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
			this->velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
				"~/body_velocity", rclcpp::QoS{20},
				[this](const geometry_msgs::msg::TwistStamped& m) { this->on_velocity(m); }
			);
			this->posterior_sub_ = this->create_subscription<BeliefArray>(
				posterior_topic, rclcpp::QoS{10}, [this](const BeliefArray& m) { this->on_posterior(m); }
			);
			// sotoba は定義上の初期姿勢を transient local で出している
			this->initial_sub_ = this->create_subscription<BeliefArray>(
				initial_topic, rclcpp::QoS{1}.transient_local(), [this](const BeliefArray& m) { this->on_initial(m); }
			);
			this->initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
				"/initialpose", rclcpp::QoS{1},
				[this](const geometry_msgs::msg::PoseWithCovarianceStamped& m) { this->on_initialpose(m); }
			);
			this->reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reset",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					res->success = this->reset_to_initial("~/reset");
					res->message = res->success ? "reset to the initial pose" : "initial pose is not known yet";
				}
			);

			// --- 出力 ---
			this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS{10});
			this->prior_pub_ = this->create_publisher<BeliefArray>(prior_topic, rclcpp::QoS{10});
			this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

			this->timer_ = rclcpp::create_timer(
				this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { this->on_timer(); }
			);
			RCLCPP_INFO(this->get_logger(), "waiting for %s and TF %s -> %s",
				initial_topic.c_str(), this->base_frame_.c_str(), this->lidar_frame_.c_str());
		}

	private:
		void load_params() {
			const auto d = [this](const char* name) { return this->get_parameter(name).as_double(); };
			FilterParams fp{};
			fp.tau_linear = d("filter.tau_linear");
			fp.tau_angular = d("filter.tau_angular");
			fp.velocity_noise_linear = d("filter.velocity_noise_linear");
			fp.velocity_noise_angular = d("filter.velocity_noise_angular");
			fp.input_sigma_linear = d("filter.input_sigma_linear");
			fp.input_sigma_angular = d("filter.input_sigma_angular");
			fp.pose_noise_linear = d("filter.pose_noise_linear");
			fp.pose_noise_angular = d("filter.pose_noise_angular");
			fp.gate_sigma = d("filter.gate_sigma");
			fp.max_horizon = d("filter.max_horizon");
			this->filter_.set_params(fp);

			this->input_delay_ = d("input_delay");
			this->measurement_timeout_ = d("measurement_timeout");
			this->initial_sigma_xy_ = d("initial_sigma_xy");
			this->initial_sigma_yaw_ = d("initial_sigma_yaw");
			this->subtract_prior_ = this->get_parameter("subtract_prior").as_bool();
			this->measurement_floor_ = Vec3{
				std::pow(d("measurement_floor_xy"), 2), std::pow(d("measurement_floor_xy"), 2),
				std::pow(d("measurement_floor_yaw"), 2)}.asDiagonal();
			this->inflate_after_failures_ = static_cast<int>(this->get_parameter("inflate_after_failures").as_int());
			this->inflate_cov_ = Vec3{
				std::pow(d("inflate_sigma_xy"), 2), std::pow(d("inflate_sigma_xy"), 2),
				std::pow(d("inflate_sigma_yaw"), 2)}.asDiagonal();
			this->track_relations_ = this->get_parameter("track_relations").as_bool();
			this->prior_noise_ = state_estimator::PriorNoise{
				.sigma_z = d("prior.sigma_z"),
				.sigma_tilt = d("prior.sigma_tilt"),
				.relation_sigma_xy = d("prior.relation_sigma_xy"),
				.relation_sigma_yaw = d("prior.relation_sigma_yaw"),
			};
		}

		// ---------------------------------------------------------------- 入力

		void on_velocity(const geometry_msgs::msg::TwistStamped& m) {
			const rclcpp::Time stamp = rclcpp::Time{m.header.stamp}.nanoseconds() == 0 ? this->now() : rclcpp::Time{m.header.stamp};
			this->filter_.add_input(
				stamp.seconds() + this->input_delay_, Vec3{m.twist.linear.x, m.twist.linear.y, m.twist.angular.z}
			);
		}

		/// sotoba の定義上の初期姿勢。物体同士の位置関係と、初期化の姿勢に使う
		void on_initial(const BeliefArray& m) {
			this->names_ = m.names;
			this->initial_means_.clear();
			for (std::size_t i = 0; i < m.names.size() && i < m.means.size(); ++i) {
				this->initial_means_.push_back(to_se3(m.means[i]));
				if (m.names[i] == this->field_object_) {
					this->field_index_ = i;
				}
			}
			if (!this->field_index_) {
				RCLCPP_ERROR(this->get_logger(), "initial beliefs have no object named '%s'", this->field_object_.c_str());
				return;
			}
			this->reset_relations();
			RCLCPP_INFO(this->get_logger(), "got %zu object(s) from sotoba", this->names_.size());
		}

		void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped& m) {
			if (!m.header.frame_id.empty() && m.header.frame_id != this->field_frame_) {
				RCLCPP_WARN(this->get_logger(), "/initialpose is in '%s', not '%s'; ignored",
					m.header.frame_id.c_str(), this->field_frame_.c_str());
				return;
			}
			const auto& q = m.pose.pose.orientation;
			const Vec3 pose{m.pose.pose.position.x, m.pose.pose.position.y,
				std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
			Mat3 cov = Mat3::Zero();
			cov(0, 0) = m.pose.covariance[0];
			cov(1, 1) = m.pose.covariance[7];
			cov(2, 2) = m.pose.covariance[35];
			if (!(cov.trace() > 0.0)) {
				cov = this->initial_cov();
			}
			this->reset_to(pose, cov, "/initialpose");
		}

		void on_posterior(const BeliefArray& m) {
			if (!this->field_index_ || !this->T_base_lidar_ || !this->filter_.initialized()) {
				return;
			}
			const rclcpp::Time stamp{m.header.stamp};
			// リセット前の事前から作られた事後は使わない
			if (this->reset_stamp_ && stamp < *this->reset_stamp_) {
				return;
			}

			std::unordered_map<std::string, std::size_t> index{};
			for (std::size_t i = 0; i < m.names.size(); ++i) {
				index.emplace(m.names[i], i);
			}
			const auto it = index.find(this->field_object_);
			if (it == index.end() || it->second >= m.means.size()) {
				return;
			}
			const std::size_t fi = it->second;
			const auto updated = [&](const std::size_t i) {
				return m.status.empty() || (i < m.status.size() && m.status[i] == status_updated);
			};
			if (!updated(fi)) {
				this->on_failure("sotoba could not update the field");
				return;
			}

			Belief posterior{to_se3(m.means[fi]), Mat6::Zero()};
			bool has_information = false;
			for (const auto& block : m.blocks) {
				if (block.i == fi && block.j == fi) {
					posterior.information = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>{block.information.data()};
					has_information = true;
				}
			}
			if (!has_information) {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "posterior has no information block for the field");
				return;
			}

			// 自分の事前を差し引く。sotoba はスキャン時刻以前で最新の事前を使うので、同じものを選ぶ
			Belief scan = posterior;
			if (this->subtract_prior_) {
				const SentPrior* used = nullptr;
				for (auto p = this->sent_.rbegin(); p != this->sent_.rend(); ++p) {
					if (p->stamp <= stamp) {
						used = &*p;
						break;
					}
				}
				if (used != nullptr) {
					scan = state_estimator::remove_prior(posterior, used->field);
				}
			}

			const auto meas = state_estimator::planar_measurement(scan, *this->T_base_lidar_, this->measurement_floor_);
			const auto result = this->filter_.update(stamp.seconds(), meas.pose, meas.covariance);
			++this->counts_[static_cast<std::size_t>(result)];
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"posteriors: %zu accepted, %zu gated, %zu out of order; last z=(%.3f, %.3f, %.3f) sigma=(%.4f, %.4f, %.4f)",
				this->counts_[0], this->counts_[2], this->counts_[1], meas.pose(0), meas.pose(1), meas.pose(2),
				std::sqrt(meas.covariance(0, 0)), std::sqrt(meas.covariance(1, 1)), std::sqrt(meas.covariance(2, 2)));
			switch (result) {
				case UpdateResult::accepted:
					this->last_accepted_ = this->now();
					this->failures_ = 0;
					if (!this->localized_) {
						this->localized_ = true;
						RCLCPP_INFO(this->get_logger(), "localized at (%.3f, %.3f, %.3f)", meas.pose(0), meas.pose(1), meas.pose(2));
					}
					break;
				case UpdateResult::gated:
					this->on_failure("pose measurement rejected by the gate");
					return;
				case UpdateResult::out_of_order:
					RCLCPP_DEBUG(this->get_logger(), "out-of-order posterior dropped");
					return;
				case UpdateResult::not_initialized:
					return;
			}

			// フィールド以外の物体の、フィールドに対する位置関係を覚え直す (動かされても追いかける)
			if (this->track_relations_) {
				const SE3 T_field_lidar = posterior.mean.inverse();
				for (std::size_t i = 0; i < m.names.size() && i < m.means.size(); ++i) {
					if (i == fi || !updated(i)) {
						continue;
					}
					for (std::size_t k = 0; k < this->names_.size(); ++k) {
						if (this->names_[k] == m.names[i]) {
							this->relations_[k] = T_field_lidar * to_se3(m.means[i]);
						}
					}
				}
			}
		}

		void on_failure(const char* reason) {
			++this->failures_;
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", reason);
			if (this->inflate_after_failures_ > 0 && this->failures_ >= this->inflate_after_failures_) {
				// 位置は捨てず、不確かさだけ大きくする (sotoba が広めに探し直せるように)
				this->filter_.inflate(this->inflate_cov_);
				this->failures_ = 0;
				RCLCPP_WARN(this->get_logger(), "%d consecutive failures; inflating the uncertainty", this->inflate_after_failures_);
			}
		}

		// ---------------------------------------------------------------- 初期化とリセット

		auto initial_cov() const -> Mat3 {
			return Vec3{this->initial_sigma_xy_ * this->initial_sigma_xy_, this->initial_sigma_xy_ * this->initial_sigma_xy_,
				this->initial_sigma_yaw_ * this->initial_sigma_yaw_}.asDiagonal();
		}

		void reset_relations() {
			const SE3 T_field_lidar0 = this->initial_means_[*this->field_index_].inverse();
			this->relations_.clear();
			for (const auto& M : this->initial_means_) {
				this->relations_.push_back(T_field_lidar0 * M);
			}
		}

		auto reset_to_initial(const char* why) -> bool {
			if (!this->field_index_ || !this->T_base_lidar_) {
				return false;
			}
			this->reset_relations();
			const Vec3 pose = state_estimator::base_pose_from_field(this->initial_means_[*this->field_index_], *this->T_base_lidar_);
			this->reset_to(pose, this->initial_cov(), why);
			return true;
		}

		void reset_to(const Vec3& pose, const Mat3& cov, const char* why) {
			const auto now = this->now();
			this->filter_.initialize(now.seconds(), pose, cov);
			// これより前のスキャンの事後は、リセット前の事前から作られているので捨てる
			this->reset_stamp_ = now;
			this->sent_.clear();
			this->localized_ = false;
			this->last_accepted_.reset();
			this->failures_ = 0;
			RCLCPP_INFO(this->get_logger(), "reset (%s) to (%.3f, %.3f, %.3f)", why, pose(0), pose(1), pose(2));
		}

		void try_initialize() {
			if (!this->T_base_lidar_) {
				try {
					const auto tf = this->tf_buffer_->lookupTransform(this->base_frame_, this->lidar_frame_, tf2::TimePointZero);
					const auto& t = tf.transform.translation;
					const auto& r = tf.transform.rotation;
					this->T_base_lidar_ = SE3{Sophus::SO3d{Eigen::Quaterniond{r.w, r.x, r.y, r.z}.normalized()},
						Eigen::Vector3d{t.x, t.y, t.z}};
				} catch (const tf2::TransformException&) {
					return;
				}
			}
			if (this->field_index_ && !this->filter_.initialized()) {
				this->reset_to_initial("initial pose from sotoba");
			}
		}

		// ---------------------------------------------------------------- 出力

		void on_timer() {
			if (!this->filter_.initialized()) {
				this->try_initialize();
				if (!this->filter_.initialized()) {
					return;
				}
			}
			const auto now = this->now();
			this->filter_.advance_anchor(now.seconds());
			const auto est = this->filter_.predict(now.seconds());
			if (!est) {
				return;
			}
			const Vec3 pose = est->x.head<3>();
			const Mat3 pose_cov = est->P.topLeftCorner<3, 3>();

			// sotoba への事前は、観測が途切れていても出し続ける (不確かさが育つので探し直せる)
			this->publish_prior(now, pose, pose_cov);

			const bool fresh = this->last_accepted_ && (now - *this->last_accepted_).seconds() <= this->measurement_timeout_;
			if (!fresh) {
				// 自己位置が取れていないときは出さない (下流の tracker がタイムアウトで止まる)
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s",
					this->localized_ ? "no pose measurement for a while" : "waiting for the first pose measurement");
				return;
			}
			this->publish_state(now, *est);
		}

		void publish_prior(const rclcpp::Time& now, const Vec3& pose, const Mat3& pose_cov) {
			BeliefArray msg{};
			msg.header.stamp = now;
			msg.header.frame_id = this->lidar_frame_;
			SentPrior sent{now, {}};
			for (std::size_t i = 0; i < this->names_.size(); ++i) {
				auto noise = this->prior_noise_;
				if (i == *this->field_index_) {
					noise.relation_sigma_xy = 0.0;
					noise.relation_sigma_yaw = 0.0;
				}
				const Belief b = state_estimator::make_prior(pose, pose_cov, *this->T_base_lidar_, this->relations_[i], noise);
				msg.names.push_back(this->names_[i]);
				msg.means.push_back(to_pose(b.mean));
				sotoba_ros::msg::InformationBlock block{};
				block.i = static_cast<std::uint8_t>(i);
				block.j = static_cast<std::uint8_t>(i);
				Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>{block.information.data()} = b.information;
				msg.blocks.push_back(block);
				if (i == *this->field_index_) {
					sent.field = b;
				}
			}
			this->prior_pub_->publish(msg);

			this->sent_.push_back(sent);
			while (this->sent_.size() > 1 && (now - this->sent_.front().stamp).seconds() > 1.0) {
				this->sent_.pop_front();
			}
		}

		void publish_state(const rclcpp::Time& now, const state_estimator::State& est) {
			nav_msgs::msg::Odometry odom{};
			odom.header.stamp = now;
			odom.header.frame_id = this->field_frame_;
			odom.child_frame_id = this->base_frame_;
			odom.pose.pose.position.x = est.x(0);
			odom.pose.pose.position.y = est.x(1);
			odom.pose.pose.orientation = yaw_quaternion(est.x(2));
			// (x, y, z, roll, pitch, yaw) の 6x6。平面の成分だけ埋める
			const int idx[3]{0, 1, 5};
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					odom.pose.covariance[idx[i] * 6 + idx[j]] = est.P(i, j);
					odom.twist.covariance[idx[i] * 6 + idx[j]] = est.P(3 + i, 3 + j);
				}
			}
			odom.twist.twist.linear.x = est.x(3);
			odom.twist.twist.linear.y = est.x(4);
			odom.twist.twist.angular.z = est.x(5);
			this->odom_pub_->publish(odom);

			if (this->publish_tf_) {
				geometry_msgs::msg::TransformStamped tf{};
				tf.header = odom.header;
				tf.child_frame_id = this->base_frame_;
				tf.transform.translation.x = est.x(0);
				tf.transform.translation.y = est.x(1);
				tf.transform.rotation = odom.pose.pose.orientation;
				this->tf_broadcaster_->sendTransform(tf);
			}
		}

		struct SentPrior {
			rclcpp::Time stamp;
			Belief field;
		};

		// 設定
		std::string field_frame_{};
		std::string base_frame_{};
		std::string lidar_frame_{};
		std::string field_object_{};
		bool publish_tf_{true};
		double input_delay_{};
		double measurement_timeout_{};
		double initial_sigma_xy_{};
		double initial_sigma_yaw_{};
		bool subtract_prior_{true};
		Mat3 measurement_floor_{Mat3::Zero()};
		int inflate_after_failures_{};
		Mat3 inflate_cov_{Mat3::Zero()};
		bool track_relations_{true};
		state_estimator::PriorNoise prior_noise_{};

		// 状態
		Filter filter_{};
		std::optional<SE3> T_base_lidar_{};
		std::vector<std::string> names_{};
		std::vector<SE3> initial_means_{};
		std::optional<std::size_t> field_index_{};
		/// 各物体のフィールド座標系での姿勢
		std::vector<SE3> relations_{};
		std::deque<SentPrior> sent_{};
		std::optional<rclcpp::Time> reset_stamp_{};
		std::optional<rclcpp::Time> last_accepted_{};
		bool localized_{false};
		int failures_{};
		std::array<std::size_t, 4> counts_{};

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_{};
		rclcpp::Subscription<BeliefArray>::SharedPtr posterior_sub_{};
		rclcpp::Subscription<BeliefArray>::SharedPtr initial_sub_{};
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_{};
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{};
		rclcpp::Publisher<BeliefArray>::SharedPtr prior_pub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
		rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateEstimatorNode>());
	rclcpp::shutdown();
	return 0;
}
