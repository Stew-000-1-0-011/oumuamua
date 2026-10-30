#pragma once

/// 作業の姿勢の計算と、機体と障害物の間隔の確認。ROS に依存しない。
///
/// 座標はフィールド座標系 (原点はフィールド中心、+x がスタート -> ビンゴ、+y が左)。
/// 機体は機体座標系 (x 前・y 左) の多角形 (フットプリント) で表す。把持機構のように、持ち上げて壁やブロックの
/// 上を通る部分は入れない。

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

	/// 機体座標系の点
	struct Point final {
		double x{};
		double y{};
	};

	/// 機体の外形 (機体座標系の多角形の頂点。向きはどちら回りでもよい)
	using Footprint = std::vector<Point>;

	struct Field final {
		Box bounds{};               ///< 走れる範囲 (外周壁の内側の面)
		std::vector<Box> obstacles; ///< 壁・棚・残っているブロックなど
	};

	/// 機体座標系で offset にある道具 (把持機構) を、向き yaw でフィールドの target に合わせるときの機体の姿勢
	auto pose_for_tool(double target_x, double target_y, double yaw, double offset_x, double offset_y) -> Pose2;

	/// 姿勢 pose から、機体座標系で (dx, dy) だけ動いた姿勢
	auto offset_pose(const Pose2& pose, double dx, double dy) -> Pose2;

	/// 姿勢 pose の機体 (外形 footprint) と、一番近い障害物・外周との間隔 [m]。重なっていれば負
	/// (外形の辺を 1 cm ごとに見るので、それより細かい違いは見ない)
	auto clearance(const Field& field, const Pose2& pose, const Footprint& footprint) -> double;

	struct SweepCheck final {
		double clearance{}; ///< 一番狭いところの間隔 [m]
		double x{};         ///< そのときの機体の姿勢
		double y{};
		double yaw{};
	};

	/// poses を順に直線で動くとき (向きは区間の中で一様に回す) の、一番狭いところ。
	/// 機体の点が step [m] より大きく動かない間隔で見る
	auto sweep_clearance(const Field& field, const std::vector<Pose2>& poses, const Footprint& footprint, double step) -> SweepCheck;

	/// 平らな配列 [x0, y0, x1, y1, ...] を外形にする。3 点未満か長さが奇数なら std::invalid_argument
	auto footprint_from_flat(const std::vector<double>& flat, const std::string& what) -> Footprint;

	/// 平らな配列 [xmin, ymin, xmax, ymax, ...] を箱にする。長さが 4 の倍数でなければ std::invalid_argument
	auto boxes_from_flat(const std::vector<double>& flat, const std::string& what) -> std::vector<Box>;
} // namespace task_commander
