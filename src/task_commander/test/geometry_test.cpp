/// @file geometry_test.cpp
/// 作業の姿勢と間隔の確認のテスト (ROS不要)。

#include <cmath>
#include <cstdio>
#include <numbers>
#include <source_location>
#include <stdexcept>

#include "task_commander/geometry.hpp"

namespace {
	using namespace task_commander;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol) -> bool { return std::abs(a - b) <= tol; }

	void test_pose_for_tool() {
		// 機体の後ろ 0.4 m・左 0.1 m の道具を、+y 向き (yaw = pi/2) で (1, 0) に合わせる。
		// +y を向くと機体の後ろは -y、左は -x なので、機体は (1.1, 0.4)
		const auto p = pose_for_tool(1.0, 0.0, std::numbers::pi / 2, -0.4, 0.1);
		check(near(p.x, 1.1, 1e-12) && near(p.y, 0.4, 1e-12) && near(p.yaw, std::numbers::pi / 2, 1e-12), "tool lands on the target");
		const auto q = offset_pose(p, -0.4, 0.1);
		check(near(q.x, 1.0, 1e-12) && near(q.y, 0.0, 1e-12), "offset_pose is the inverse");
	}

	/// 一辺 2 * half の正方形の外形 (中心が機体の中心)
	auto square(const double half) -> Footprint { return {{-half, -half}, {half, -half}, {half, half}, {-half, half}}; }

	void test_clearance() {
		const Field field{.bounds = {-1.0, -1.0, 1.0, 1.0}, .obstacles = {{0.5, -0.1, 0.6, 0.1}}};
		check(near(clearance(field, {0.0, 0.0, 0.0}, square(0.3)), 0.2, 1e-9), "distance from the outline to an obstacle");
		check(near(clearance(field, {0.0, 0.8, 0.0}, square(0.1)), 0.1, 1e-9), "distance to the bounds");
		check(clearance(field, {0.55, 0.0, 0.0}, square(0.1)) < 0.0, "overlapping is negative");
		// 箱が外形の中にすっぽり入っていても重なりとみなす (外形の辺の点は箱に触れない)
		check(clearance(field, {0.55, 0.0, 0.0}, square(0.3)) < 0.0, "an obstacle inside the outline is overlapping");
		// 外形は向きで回る: 細長い外形 (前に 0.4 m) は、+x を向くと箱に届き、+y を向くと届かない
		const Footprint stick{{-0.05, -0.05}, {0.4, -0.05}, {0.4, 0.05}, {-0.05, 0.05}};
		check(near(clearance(field, {0.0, 0.0, 0.0}, stick), 0.1, 1e-9), "the outline follows the heading (+x)");
		check(clearance(field, {0.0, 0.0, std::numbers::pi / 2}, stick) > 0.3, "the outline follows the heading (+y)");
	}

	void test_sweep() {
		const Field field{.bounds = {-1.0, -1.0, 1.0, 1.0}, .obstacles = {{-0.05, -0.05, 0.05, 0.05}}};
		const auto c = sweep_clearance(field, {{-0.5, 0.3, 0.0}, {0.5, 0.3, 0.0}}, square(0.1), 0.01);
		check(near(c.clearance, 0.15, 1e-9) && std::abs(c.x) <= 0.15 + 1e-9, "the narrowest point of a straight move");
		// その場で回る: 前に 0.4 m の外形が -y から +y へ (左回りに) 回ると、途中で +x の箱に当たる
		const Field side{.bounds = {-1.0, -1.0, 1.0, 1.0}, .obstacles = {{0.3, -0.05, 0.35, 0.05}}};
		const Footprint stick{{-0.05, -0.02}, {0.4, -0.02}, {0.4, 0.02}, {-0.05, 0.02}};
		const auto turn = sweep_clearance(side, {{0.0, 0.0, -std::numbers::pi / 2}, {0.0, 0.0, std::numbers::pi / 2 - 0.01}}, stick, 0.01);
		check(turn.clearance < 0.0 && std::abs(turn.yaw) < 0.1, "turning in place sweeps the outline");
	}

	void test_footprint_from_flat() {
		const auto f = footprint_from_flat({0, 1, 2, 3, 4, 5}, "test");
		check(f.size() == 3 && f[2].x == 4 && f[2].y == 5, "flat lists become points");
		bool threw = false;
		try {
			footprint_from_flat({0, 1, 2, 3}, "test");
		} catch (const std::invalid_argument&) {
			threw = true;
		}
		check(threw, "fewer than 3 points is an error");
	}

	void test_boxes_from_flat() {
		const auto boxes = boxes_from_flat({0, 1, 2, 3, 4, 5, 6, 7}, "test");
		check(boxes.size() == 2 && boxes[1].xmin == 4 && boxes[1].ymax == 7, "flat lists become boxes");
		bool threw = false;
		try {
			boxes_from_flat({0, 1, 2}, "test");
		} catch (const std::invalid_argument&) {
			threw = true;
		}
		check(threw, "a length that is not a multiple of 4 is an error");
	}
} // namespace

auto main() -> int {
	test_pose_for_tool();
	test_clearance();
	test_sweep();
	test_footprint_from_flat();
	test_boxes_from_flat();
	if (failures == 0) {
		std::printf("all passed\n");
	}
	return failures == 0 ? 0 : 1;
}
