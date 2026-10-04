#pragma once
/// @file plant.hpp
/// 全方向移動ロボットの平面運動 (シミュレーション用)。ROS非依存。
///
/// 機体速度の指令 (機体座標系) に、軸ごとの 1 次遅れと加速度制限で追従し、
/// フィールド座標系で姿勢を積分する。

namespace oumuamua_sim {
	/// フィールド座標系での姿勢
	struct Pose2 {
		double x{};
		double y{};
		double yaw{};
	};

	/// 機体座標系の速度
	struct Twist2 {
		double vx{};
		double vy{};
		double omega{};
	};

	struct PlantParams {
		/// 指令への追従の時定数 [s]。<= 0 なら遅れなし
		double tau_linear{0.1};
		double tau_angular{0.1};
		/// 加速度の上限。<= 0 で無効。並進はベクトルの大きさで制限する
		double max_accel_linear{0.0};   ///< [m/s^2]
		double max_accel_angular{0.0};  ///< [rad/s^2]
		/// 実際に出る速度 / 指令 (滑りや車輪径の誤差の模擬)。1 で理想
		double velocity_scale_linear{1.0};
		double velocity_scale_angular{1.0};
	};

	class Plant {
	public:
		Plant(const PlantParams& params, const Pose2& initial) : params_{params}, pose_{initial} {}

		/// dt [s] 進める。cmd は機体座標系の速度指令
		void step(const Twist2& cmd, double dt);

		auto pose() const -> const Pose2& { return this->pose_; }
		/// 機体座標系の実際の速度
		auto velocity() const -> const Twist2& { return this->velocity_; }

		void set_params(const PlantParams& params) { this->params_ = params; }
		void reset(const Pose2& pose) {
			this->pose_ = pose;
			this->velocity_ = {};
		}

	private:
		PlantParams params_;
		Pose2 pose_;
		Twist2 velocity_{};
	};

	/// 角度を [-pi, pi) に折り返す
	auto wrap_angle(double a) -> double;
}
