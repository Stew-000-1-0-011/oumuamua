#pragma once
/// @file trajectory.hpp
/// 経路 (点列) に速度計画を付けて、時刻ごとの目標にする。ROS非依存。
///
/// 経路は機体の姿勢 (x, y, yaw) の点列で、向き yaw は進行方向と無関係に決めてよい (全方向移動)。
/// 速度は、接線方向の加減速・横加速度・角速度の上限を守る範囲でできるだけ速く、両端で止まるように決める。

#include <iosfwd>
#include <vector>

namespace path_follower {
	/// 経路上の1点 (フィールド座標系)。yaw は連続に (±pi で折り返さずに) 並べること
	struct PathPoint {
		double x{};
		double y{};
		double yaw{};
	};

	struct Limits {
		double max_speed{0.5};          ///< [m/s]
		double max_accel{1.0};          ///< [m/s^2] 接線方向の加減速
		double max_lateral_accel{1.0};  ///< [m/s^2] 曲がるときの横加速度 (v^2 * 曲率)
		double max_yaw_rate{1.5};       ///< [rad/s]
	};

	/// ある時刻の目標。速度はフィールド座標系
	struct State {
		double x{};
		double y{};
		double yaw{};
		double vx{};
		double vy{};
		double omega{};
		double s{};          ///< 始点からの道のり [m]
		bool finished{};     ///< 終点に着いた (速度は 0)
	};

	class Trajectory final {
	public:
		/// 点が 2 つ未満、位置の変わらない点が続く、上限が正でない、のいずれかなら std::invalid_argument
		Trajectory(std::vector<PathPoint> points, const Limits& limits);

		auto duration() const noexcept -> double { return this->time_.back(); }
		auto length() const noexcept -> double { return this->s_.back(); }
		auto points() const noexcept -> const std::vector<PathPoint>& { return this->points_; }

		/// 走り始めてから t 秒後の目標。t は [0, duration] に丸める
		auto sample(double t) const -> State;

	private:
		std::vector<PathPoint> points_{};
		std::vector<double> s_{};      ///< 各点までの道のり
		std::vector<double> speed_{};  ///< 各点での速さ
		std::vector<double> time_{};   ///< 各点に着く時刻
	};

	/// 頂点 (x, y, yaw) を順に直線でつないだ経路を、spacing [m] 間隔の点にする。
	/// 向きは各辺の上で一定の割合で回す。頂点の yaw は、前の頂点から近いほうへ回るように ±2pi を足し直す。
	/// 位置が同じ頂点が続くとき、向きも同じなら飛ばし、違えば (その場での旋回になるので) std::invalid_argument。
	/// 点が 2 つ未満になるとき (全部同じ位置) も std::invalid_argument
	auto polyline_path(const std::vector<PathPoint>& vertices, double spacing) -> std::vector<PathPoint>;

	/// CSV (1行に x, y, yaw) を読む。'#' で始まる行、空行、数値でない先頭行 (見出し) は飛ばす。
	/// 数値として読めない行があれば std::invalid_argument
	auto load_path_csv(std::istream& in) -> std::vector<PathPoint>;
}
