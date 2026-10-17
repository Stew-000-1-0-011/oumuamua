/// 機体速度を積分してオドメトリを出すノード。
///
/// - sub `~/body_velocity` (geometry_msgs/TwistStamped、機体座標系): chassis_node や robot_sim の機体速度
/// - pub `~/odom` (nav_msgs/Odometry): odom_frame から見た base_frame の姿勢と、機体座標系の速度
/// - TF odom_frame -> base_frame (publish_tf)
///
/// 姿勢は、前のメッセージからの区間を両端の速度の平均と中点の向きで積分する (台形則)。時刻は TwistStamped の stamp。

#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace {
	class BodyOdometry final : public rclcpp::Node {
	public:
		BodyOdometry() : rclcpp::Node("body_odometry") {
			this->odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
			this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
			this->publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
			// 1 秒あたりに増える不確かさではなく、各メッセージに付ける固定の共分散 (下流の目安)
			this->pose_variance_ = this->declare_parameter<double>("pose_variance", 1e-4);
			this->twist_variance_ = this->declare_parameter<double>("twist_variance", 1e-3);
			// 時刻が飛んだら (止まっていた・再生し直した) 積分しない [s]
			this->max_dt_ = this->declare_parameter<double>("max_dt", 0.5);

			this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS{10});
			if (this->publish_tf_) {
				this->tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
			}
			this->sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
				"~/body_velocity",
				rclcpp::QoS{20},
				[this](const geometry_msgs::msg::TwistStamped& m) { this->on_velocity(m); }
			);
		}

	private:
		void on_velocity(const geometry_msgs::msg::TwistStamped& m) {
			const rclcpp::Time stamp{m.header.stamp, RCL_ROS_TIME};
			const double vx = m.twist.linear.x;
			const double vy = m.twist.linear.y;
			const double w = m.twist.angular.z;
			if (this->has_last_) {
				const double dt = (stamp - this->last_stamp_).seconds();
				if (dt > 0.0 && dt <= this->max_dt_) {
					// 区間の速度は両端の平均 (台形則)。区間の中点の向きで回して足す
					const double avx = 0.5 * (this->vx_ + vx);
					const double avy = 0.5 * (this->vy_ + vy);
					const double aw = 0.5 * (this->w_ + w);
					const double mid = this->yaw_ + 0.5 * aw * dt;
					this->x_ += (std::cos(mid) * avx - std::sin(mid) * avy) * dt;
					this->y_ += (std::sin(mid) * avx + std::cos(mid) * avy) * dt;
					this->yaw_ = std::remainder(this->yaw_ + aw * dt, 2.0 * std::numbers::pi);
				} else if (dt <= 0.0) {
					return;
				}
			}
			this->has_last_ = true;
			this->last_stamp_ = stamp;
			this->vx_ = vx;
			this->vy_ = vy;
			this->w_ = w;
			this->publish(m.header.stamp);
		}

		void publish(const builtin_interfaces::msg::Time& stamp) {
			const double qz = std::sin(0.5 * this->yaw_);
			const double qw = std::cos(0.5 * this->yaw_);

			nav_msgs::msg::Odometry odom{};
			odom.header.stamp = stamp;
			odom.header.frame_id = this->odom_frame_;
			odom.child_frame_id = this->base_frame_;
			odom.pose.pose.position.x = this->x_;
			odom.pose.pose.position.y = this->y_;
			odom.pose.pose.orientation.z = qz;
			odom.pose.pose.orientation.w = qw;
			odom.twist.twist.linear.x = this->vx_;
			odom.twist.twist.linear.y = this->vy_;
			odom.twist.twist.angular.z = this->w_;
			for (const int i : {0, 7, 14, 21, 28, 35}) {
				odom.pose.covariance[static_cast<std::size_t>(i)] = this->pose_variance_;
				odom.twist.covariance[static_cast<std::size_t>(i)] = this->twist_variance_;
			}
			this->odom_pub_->publish(odom);

			if (this->tf_) {
				geometry_msgs::msg::TransformStamped t{};
				t.header = odom.header;
				t.child_frame_id = this->base_frame_;
				t.transform.translation.x = this->x_;
				t.transform.translation.y = this->y_;
				t.transform.rotation.z = qz;
				t.transform.rotation.w = qw;
				this->tf_->sendTransform(t);
			}
		}

		std::string odom_frame_{};
		std::string base_frame_{};
		bool publish_tf_{true};
		double pose_variance_{};
		double twist_variance_{};
		double max_dt_{};

		bool has_last_{false};
		rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
		double x_{0.0};
		double y_{0.0};
		double yaw_{0.0};
		double vx_{0.0};
		double vy_{0.0};
		double w_{0.0};

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{};
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_{};
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BodyOdometry>());
	rclcpp::shutdown();
	return 0;
}
