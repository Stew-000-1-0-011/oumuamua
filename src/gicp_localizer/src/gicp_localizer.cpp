/// 2D LiDAR のスキャンを、フィールドの点群地図に small_gicp (GICP) で位置合わせして自己位置を出すノード。
///
/// - 地図: `map_file` の PCD (oumuamua_sim の make_field_map が作る、走査面上の壁の点。z = 0)
/// - 初期値: スキャンの時刻の TF `field_frame -> base_frame`。無ければ、自分が最後に求めた
///   `field_frame -> odom_frame` と、その時刻の `odom_frame -> base_frame` (body_odometry) から作る。
///   どちらも無ければ `initial_pose`
/// - 出力: `~/pose` (PoseWithCovarianceStamped、フィールド座標系の機体の姿勢、スキャンの時刻)。
///   `publish_tf` なら TF `field_frame -> odom_frame` も出す (AMCL と同じ形。少し先の時刻を付ける)
///
/// 2D の点は 1 本の直線上に並ぶので、そのままでは GICP の共分散 (近傍の広がり) で
/// 「壁の法線方向」と「z 方向」の区別がつかない。地図とスキャンの点を z = ±layer_offset にも複製して、
/// 近傍が鉛直な面になるようにする。z・roll・pitch は RestrictDoFFactor で固定する。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace {
	using Cloud = small_gicp::PointCloud;
	using Tree = small_gicp::KdTree<Cloud>;
	using Reg = small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP, small_gicp::RestrictDoFFactor>;

	auto planar(const double x, const double y, const double yaw) -> Eigen::Isometry3d {
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
		T.translation() = Eigen::Vector3d{x, y, 0.0};
		T.linear() = Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
		return T;
	}

	auto yaw_of(const Eigen::Isometry3d& T) -> double { return std::atan2(T.linear()(1, 0), T.linear()(0, 0)); }

	/// 平面の成分だけ残す (z・roll・pitch を 0 にする)
	auto flatten(const Eigen::Isometry3d& T) -> Eigen::Isometry3d {
		return planar(T.translation().x(), T.translation().y(), yaw_of(T));
	}

	auto to_isometry(const geometry_msgs::msg::Transform& t) -> Eigen::Isometry3d {
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
		T.translation() = Eigen::Vector3d{t.translation.x, t.translation.y, t.translation.z};
		T.linear() = Eigen::Quaterniond{t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z}.normalized().toRotationMatrix();
		return T;
	}

	class GicpLocalizer final : public rclcpp::Node {
	public:
		GicpLocalizer() : rclcpp::Node("gicp_localizer") {
			this->field_frame_ = this->declare_parameter<std::string>("field_frame", "field");
			this->odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
			this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
			this->publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
			this->transform_tolerance_ = this->declare_parameter<double>("transform_tolerance", 0.1);
			const auto initial = this->declare_parameter<std::vector<double>>("initial_pose", std::vector<double>{0.0, 0.0, 0.0});
			if (initial.size() == 3) {
				this->last_ = planar(initial[0], initial[1], initial[2]);
			}
			this->range_min_ = this->declare_parameter<double>("range_min", 0.05);
			this->range_max_ = this->declare_parameter<double>("range_max", 10.0);
			this->layer_offset_ = this->declare_parameter<double>("layer_offset", 0.05);
			// スキャンの点の間引き (1 なら全部使う)。計算量はほぼ点の数に比例する
			this->point_stride_ = static_cast<std::size_t>(std::max<std::int64_t>(1, this->declare_parameter<std::int64_t>("point_stride", 1)));
			this->num_neighbors_ = static_cast<int>(this->declare_parameter<std::int64_t>("num_neighbors", 10));
			this->num_threads_ = static_cast<int>(this->declare_parameter<std::int64_t>("num_threads", 2));
			// 対応点の距離の上限 [m]。粗い段 -> 細かい段の 2 段で合わせる
			this->coarse_distance_ = this->declare_parameter<double>("max_correspondence_distance_coarse", 0.3);
			this->fine_distance_ = this->declare_parameter<double>("max_correspondence_distance", 0.1);
			this->max_iterations_ = static_cast<int>(this->declare_parameter<std::int64_t>("max_iterations", 20));
			// 対応の取れた点がこの割合を切ったら、そのスキャンは使わない
			this->min_inlier_ratio_ = this->declare_parameter<double>("min_inlier_ratio", 0.3);
			// 初期値から、これより大きく動いた結果は使わない [m] / [rad]
			this->max_jump_xy_ = this->declare_parameter<double>("max_jump_xy", 0.3);
			this->max_jump_yaw_ = this->declare_parameter<double>("max_jump_yaw", 0.3);
			// 共分散の下限 (標準偏差)
			this->floor_xy_ = this->declare_parameter<double>("covariance_floor_xy", 0.002);
			this->floor_yaw_ = this->declare_parameter<double>("covariance_floor_yaw", 0.002);
			this->tf_timeout_ = this->declare_parameter<double>("tf_timeout", 0.05);

			this->load_map(this->declare_parameter<std::string>("map_file", ""));

			this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
			this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
			this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", rclcpp::QoS{10});
			this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
				this->declare_parameter<std::string>("scan_topic", "/scan"),
				rclcpp::SensorDataQoS{},
				[this](const sensor_msgs::msg::LaserScan& m) { this->on_scan(m); }
			);
			if (this->publish_tf_) {
				this->tf_timer_ = this->create_wall_timer(std::chrono::milliseconds{20}, [this] { this->publish_odom_tf(); });
			}
		}

	private:
		/// 平面の点を z = 0, ±layer_offset に並べる
		void add_layers(std::vector<Eigen::Vector4d>& out, const double x, const double y) const {
			for (const double z : {-this->layer_offset_, 0.0, this->layer_offset_}) {
				out.emplace_back(x, y, z, 1.0);
			}
		}

		void load_map(const std::string& path) {
			std::ifstream in{path};
			if (!in) {
				throw std::runtime_error("cannot open map_file '" + path + "'");
			}
			std::string line;
			bool data = false;
			std::vector<Eigen::Vector4d> points;
			while (std::getline(in, line)) {
				if (!data) {
					data = line.rfind("DATA", 0) == 0;
					continue;
				}
				std::istringstream ss{line};
				double x{};
				double y{};
				if (ss >> x >> y) {
					this->add_layers(points, x, y);
				}
			}
			this->map_ = std::make_shared<Cloud>(points);
			small_gicp::estimate_covariances_omp(*this->map_, this->num_neighbors_, this->num_threads_);
			this->tree_ = std::make_shared<Tree>(this->map_, small_gicp::KdTreeBuilderOMP{this->num_threads_});
			RCLCPP_INFO(this->get_logger(), "map: %zu points (%zu with layers) from %s", points.size() / 3, points.size(), path.c_str());
		}

		auto lookup(const std::string& parent, const std::string& child, const rclcpp::Time& stamp, const double timeout)
			-> std::optional<Eigen::Isometry3d> {
			try {
				return to_isometry(this->tf_buffer_->lookupTransform(
					parent, child, stamp, rclcpp::Duration::from_seconds(timeout)).transform);
			} catch (const tf2::TransformException&) {
				return std::nullopt;
			}
		}

		void on_scan(const sensor_msgs::msg::LaserScan& scan) {
			const rclcpp::Time stamp{scan.header.stamp};
			if (!this->T_base_laser_) {
				this->T_base_laser_ = this->lookup(this->base_frame_, scan.header.frame_id, rclcpp::Time{0, 0, RCL_ROS_TIME}, 0.0);
				if (!this->T_base_laser_) {
					RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "waiting for TF %s -> %s",
						this->base_frame_.c_str(), scan.header.frame_id.c_str());
					return;
				}
			}

			// --- 初期値 ---
			const auto odom_base = this->lookup(this->odom_frame_, this->base_frame_, stamp, this->tf_timeout_);
			Eigen::Isometry3d init = this->last_;
			if (const auto field_base = this->lookup(this->field_frame_, this->base_frame_, stamp, 0.0)) {
				init = flatten(*field_base);
			} else if (this->T_field_odom_ && odom_base) {
				init = flatten(*this->T_field_odom_ * *odom_base);
			}

			// --- スキャン -> 機体座標系の平面の点 ---
			std::vector<Eigen::Vector4d> points;
			points.reserve(scan.ranges.size() * 3);
			for (std::size_t i = 0; i < scan.ranges.size(); i += this->point_stride_) {
				const double r = scan.ranges[i];
				if (!std::isfinite(r) || r < std::max<double>(this->range_min_, scan.range_min)
					|| r > std::min<double>(this->range_max_, scan.range_max)) {
					continue;
				}
				const double a = scan.angle_min + scan.angle_increment * static_cast<double>(i);
				const Eigen::Vector3d p = *this->T_base_laser_ * Eigen::Vector3d{r * std::cos(a), r * std::sin(a), 0.0};
				this->add_layers(points, p.x(), p.y());
			}
			const std::size_t n = points.size() / 3;
			if (n < 20) {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "too few points in scan (%zu)", n);
				return;
			}
			Cloud source{points};
			small_gicp::estimate_covariances_omp(source, this->num_neighbors_, this->num_threads_);

			// --- 位置合わせ (粗 -> 細) ---
			Reg reg;
			reg.reduction.num_threads = this->num_threads_;
			reg.optimizer.max_iterations = this->max_iterations_;
			reg.general_factor.set_rotation_mask(Eigen::Array3d{0.0, 0.0, 1.0});
			reg.general_factor.set_translation_mask(Eigen::Array3d{1.0, 1.0, 0.0});
			reg.rejector.max_dist_sq = this->coarse_distance_ * this->coarse_distance_;
			auto result = reg.align(*this->map_, source, *this->tree_, init);
			reg.rejector.max_dist_sq = this->fine_distance_ * this->fine_distance_;
			result = reg.align(*this->map_, source, *this->tree_, result.T_target_source);

			const double inlier_ratio = static_cast<double>(result.num_inliers) / static_cast<double>(points.size());
			const Eigen::Isometry3d T = flatten(result.T_target_source);
			const double jump_xy = (T.translation() - init.translation()).head<2>().norm();
			const double jump_yaw = std::abs(std::remainder(yaw_of(T) - yaw_of(init), 2.0 * std::numbers::pi));
			if (inlier_ratio < this->min_inlier_ratio_ || jump_xy > this->max_jump_xy_ || jump_yaw > this->max_jump_yaw_) {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"scan rejected: inliers %.0f %%, jump %.3f m / %.3f rad", 100.0 * inlier_ratio, jump_xy, jump_yaw);
				return;
			}
			this->last_ = T;
			if (odom_base) {
				this->T_field_odom_ = T * odom_base->inverse();
			}

			// --- 共分散: H^-1 を残差の大きさで目盛り直す ---
			// small_gicp の H は (回転, 並進) の順。平面の成分 (x, y, yaw) を取り出す
			const int idx[3]{3, 4, 2};
			Eigen::Matrix3d H;
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					H(i, j) = result.H(idx[i], idx[j]);
				}
			}
			const double dof = std::max<double>(1.0, static_cast<double>(result.num_inliers) - 3.0);
			const double scale = 2.0 * result.error / dof;
			Eigen::Matrix3d cov = H.inverse() * scale;
			// 機体座標系の並進 -> フィールド座標系
			Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
			R.topLeftCorner<2, 2>() = T.linear().topLeftCorner<2, 2>();
			cov = R * cov * R.transpose();
			cov(0, 0) += this->floor_xy_ * this->floor_xy_;
			cov(1, 1) += this->floor_xy_ * this->floor_xy_;
			cov(2, 2) += this->floor_yaw_ * this->floor_yaw_;

			geometry_msgs::msg::PoseWithCovarianceStamped msg{};
			msg.header.stamp = scan.header.stamp;
			msg.header.frame_id = this->field_frame_;
			msg.pose.pose.position.x = T.translation().x();
			msg.pose.pose.position.y = T.translation().y();
			msg.pose.pose.orientation.z = std::sin(0.5 * yaw_of(T));
			msg.pose.pose.orientation.w = std::cos(0.5 * yaw_of(T));
			const int out[3]{0, 1, 5};
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					msg.pose.covariance[static_cast<std::size_t>(out[i] * 6 + out[j])] = cov(i, j);
				}
			}
			this->pose_pub_->publish(msg);
			this->publish_odom_tf();
		}

		/// field -> odom を、少し先の時刻を付けて出し続ける (odom -> base_link と合わせて最新の自己位置になる)
		void publish_odom_tf() {
			if (!this->publish_tf_ || !this->T_field_odom_) {
				return;
			}
			const Eigen::Isometry3d& T = *this->T_field_odom_;
			geometry_msgs::msg::TransformStamped tf{};
			tf.header.stamp = this->now() + rclcpp::Duration::from_seconds(this->transform_tolerance_);
			tf.header.frame_id = this->field_frame_;
			tf.child_frame_id = this->odom_frame_;
			tf.transform.translation.x = T.translation().x();
			tf.transform.translation.y = T.translation().y();
			tf.transform.rotation.z = std::sin(0.5 * yaw_of(T));
			tf.transform.rotation.w = std::cos(0.5 * yaw_of(T));
			this->tf_broadcaster_->sendTransform(tf);
		}

		std::string field_frame_{};
		std::string odom_frame_{};
		std::string base_frame_{};
		bool publish_tf_{true};
		double transform_tolerance_{};
		double range_min_{};
		double range_max_{};
		double layer_offset_{};
		std::size_t point_stride_{1};
		int num_neighbors_{};
		int num_threads_{};
		double coarse_distance_{};
		double fine_distance_{};
		int max_iterations_{};
		double min_inlier_ratio_{};
		double max_jump_xy_{};
		double max_jump_yaw_{};
		double floor_xy_{};
		double floor_yaw_{};
		double tf_timeout_{};

		std::shared_ptr<Cloud> map_{};
		std::shared_ptr<Tree> tree_{};
		std::optional<Eigen::Isometry3d> T_base_laser_{};
		std::optional<Eigen::Isometry3d> T_field_odom_{};
		Eigen::Isometry3d last_{Eigen::Isometry3d::Identity()};

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_{};
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_{};
		rclcpp::TimerBase::SharedPtr tf_timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GicpLocalizer>());
	rclcpp::shutdown();
	return 0;
}
