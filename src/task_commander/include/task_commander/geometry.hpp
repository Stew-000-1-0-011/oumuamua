#pragma once

/// 作業の姿勢の計算と、機体と障害物の間隔の確認。ROS に依存しない。
///
/// 座標はフィールド座標系 (原点はフィールド中心、+x がスタート -> ビンゴ、+y が左)。
/// 機体は半径 radius の円とみなす (把持機構のように円からはみ出す部分は見ない)。

#include <string>
#include <vector>

namespace task_commander {
	struct Pose2 final {
		double x{};
		double y{};
		double yaw{};
	};

	/// 軸に平行な箱
	struct Box final {
		double xmin{};
		double ymin{};
		double xmax{};
		double ymax{};
	};

	struct Field final {
		Box bounds{};               ///< 走れる範囲 (外周壁の内側の面)
		std::vector<Box> obstacles; ///< 壁・棚・残っているブロックなど
	};

	/// 機体座標系で offset にある道具 (把持機構) を、向き yaw でフィールドの target に合わせるときの機体の姿勢
	auto pose_for_tool(double target_x, double target_y, double yaw, double offset_x, double offset_y) -> Pose2;

	/// 姿勢 pose から、機体座標系で (dx, dy) だけ動いた姿勢
	auto offset_pose(const Pose2& pose, double dx, double dy) -> Pose2;

	/// 中心 (x, y)、半径 radius の円と、一番近い障害物・外周との間隔 [m]。重なっていれば負
	auto clearance(const Field& field, double x, double y, double radius) -> double;

	struct SweepCheck final {
		double clearance{}; ///< 一番狭いところの間隔 [m]
		double x{};         ///< そのときの機体の位置
		double y{};
	};

	/// poses を順に直線で動くとき (step [m] ごとに見る) の、一番狭いところ
	auto sweep_clearance(const Field& field, const std::vector<Pose2>& poses, double radius, double step) -> SweepCheck;

	/// 平らな配列 [xmin, ymin, xmax, ymax, ...] を箱にする。長さが 4 の倍数でなければ std::invalid_argument
	auto boxes_from_flat(const std::vector<double>& flat, const std::string& what) -> std::vector<Box>;
} // namespace task_commander
