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

	void test_clearance() {
		const Field field{.bounds = {-1.0, -1.0, 1.0, 1.0}, .obstacles = {{0.5, -0.1, 0.6, 0.1}}};
		check(near(clearance(field, 0.0, 0.0, 0.3), 0.2, 1e-12), "distance to an obstacle minus the radius");
		check(near(clearance(field, 0.0, 0.8, 0.1), 0.1, 1e-12), "distance to the bounds");
		check(clearance(field, 0.55, 0.0, 0.1) < 0.0, "overlapping is negative");
		// 角: (0.6, 0.1) から (0.9, 0.5) は 0.5
		check(near(clearance(field, 0.9, 0.5, 0.0), 0.1, 1e-12), "bounds win when closer");
	}

	void test_sweep() {
		const Field field{.bounds = {-1.0, -1.0, 1.0, 1.0}, .obstacles = {{-0.05, -0.05, 0.05, 0.05}}};
		const auto c = sweep_clearance(field, {{-0.8, 0.3, 0.0}, {0.8, 0.3, 0.0}}, 0.2, 0.01);
		check(near(c.clearance, 0.05, 1e-9) && near(c.x, 0.0, 0.006), "the narrowest point of a straight move");
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
	test_boxes_from_flat();
	if (failures == 0) {
		std::printf("all passed\n");
	}
	return failures == 0 ? 0 : 1;
}
