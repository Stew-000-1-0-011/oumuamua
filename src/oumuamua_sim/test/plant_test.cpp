/// @file plant_test.cpp
/// 平面運動モデルのテスト (ROS不要)。

#include <cmath>
#include <cstdio>
#include <numbers>
#include <source_location>

#include "oumuamua_sim/plant.hpp"

namespace {
	using oumuamua_sim::Plant;
	using oumuamua_sim::PlantParams;
	using oumuamua_sim::Pose2;
	using oumuamua_sim::Twist2;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol) -> bool { return std::abs(a - b) <= tol; }

	void run(Plant& p, const Twist2& cmd, const double seconds, const double dt = 1e-3) {
		const int n = static_cast<int>(std::round(seconds / dt));
		for (int i = 0; i < n; ++i) {
			p.step(cmd, dt);
		}
	}

	void test_body_frame() {
		// 機体が +y を向いているとき、機体の前進はフィールドの +y
		Plant p{PlantParams{.tau_linear = 0.0, .tau_angular = 0.0}, Pose2{.yaw = std::numbers::pi / 2}};
		run(p, {.vx = 1.0}, 1.0);
		// 速度は前後のステップの平均で積分するので、立ち上がりの 1 ステップぶん (0.5 mm) ずれる
		check(near(p.pose().x, 0.0, 2e-3) && near(p.pose().y, 1.0, 2e-3), "body x maps to field y when yaw = pi/2");
		// 機体の左 (+vy) はフィールドの -x
		Plant q{PlantParams{.tau_linear = 0.0, .tau_angular = 0.0}, Pose2{.yaw = std::numbers::pi / 2}};
		run(q, {.vy = 1.0}, 1.0);
		check(near(q.pose().x, -1.0, 2e-3) && near(q.pose().y, 0.0, 2e-3), "body y maps to field -x when yaw = pi/2");
	}

	void test_lag() {
		// 1 次遅れ: 時定数ぶん経つと約 63% まで追従する
		Plant p{PlantParams{.tau_linear = 0.1}, Pose2{}};
		run(p, {.vx = 1.0}, 0.1);
		check(near(p.velocity().vx, 1.0 - std::exp(-1.0), 1e-3), "first-order lag after one time constant");
		run(p, {.vx = 1.0}, 1.0);
		check(near(p.velocity().vx, 1.0, 1e-3), "settles to the command");
	}

	void test_rotate_while_translating() {
		// 機体座標系で前進しながら一定角速度で回ると、円を描いて元に戻る
		Plant p{PlantParams{.tau_linear = 0.0, .tau_angular = 0.0}, Pose2{}};
		run(p, {.vx = 1.0, .omega = 1.0}, 2.0 * std::numbers::pi);
		check(near(p.pose().x, 0.0, 1e-3) && near(p.pose().y, 0.0, 1e-3), "a full circle comes back to the start");
	}

	void test_accel_limit() {
		Plant p{PlantParams{.tau_linear = 0.0, .tau_angular = 0.0, .max_accel_linear = 2.0, .max_accel_angular = 4.0}, Pose2{}};
		run(p, {.vx = 3.0, .vy = 4.0, .omega = 10.0}, 0.5);
		const double v = std::hypot(p.velocity().vx, p.velocity().vy);
		check(near(v, 1.0, 1e-6), "linear speed grows at max_accel_linear");
		check(near(p.velocity().vx / p.velocity().vy, 0.75, 1e-6), "direction of the acceleration is kept");
		check(near(p.velocity().omega, 2.0, 1e-6), "angular speed grows at max_accel_angular");
	}

	void test_velocity_scale() {
		Plant p{PlantParams{.tau_linear = 0.0, .tau_angular = 0.0, .velocity_scale_linear = 0.9}, Pose2{}};
		run(p, {.vx = 1.0}, 1.0);
		check(near(p.pose().x, 0.9, 2e-3), "velocity_scale makes the robot slower than commanded");
	}

	void test_wrap() {
		check(near(oumuamua_sim::wrap_angle(3.0 * std::numbers::pi), -std::numbers::pi, 1e-12), "wrap 3pi");
		check(near(oumuamua_sim::wrap_angle(-0.5), -0.5, 1e-12), "wrap keeps small angles");
	}
}

auto main() -> int {
	test_body_frame();
	test_lag();
	test_rotate_while_translating();
	test_accel_limit();
	test_velocity_scale();
	test_wrap();
	if (failures == 0) {
		std::printf("all plant tests passed\n");
		return 0;
	}
	std::printf("%d failure(s)\n", failures);
	return 1;
}
