/// @file lift_test.cpp
/// 昇降の、ROS に依存しない部分のテスト。

#include <cmath>
#include <cstdio>
#include <source_location>

#include "gripper/lift.hpp"

namespace {
	using namespace gripper;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol) -> bool { return std::abs(a - b) <= tol; }

	void test_parse_feedback() {
		// 角度 0x1234, 回転数 -100 (0xff9c), 電流 1000 (0x03e8), 温度 40
		const std::array<std::uint8_t, 8> data{0x12, 0x34, 0xff, 0x9c, 0x03, 0xe8, 40, 0};
		const auto fb = parse_feedback(0x203, 8, data, 3);
		check(fb.has_value(), "feedback of the motor is read");
		check(fb && fb->angle == 0x1234 && fb->rpm == -100 && fb->current == 1000 && fb->temperature == 40,
			"fields are big-endian");
		check(!parse_feedback(0x202, 8, data, 3), "other motors are ignored");
		check(!parse_feedback(0x203, 6, data, 3), "short frames are ignored");
		check(!parse_feedback(0x209, 8, data, 9), "motor numbers out of 1..8 are rejected");
	}

	void test_rotor_counter() {
		RotorCounter c{};
		check(!c.valid(), "not valid before the first angle");
		c.update(8000);
		check(c.valid() && c.revolutions() == 0.0, "the first angle is the origin");
		// 8000 -> 100 は前に 292 カウント (0 をまたぐ)
		c.update(100);
		check(near(c.revolutions(), 292.0 / 8192.0, 1e-12), "wraps forward across zero");
		// 100 -> 8000 は後ろに 292 カウント
		c.update(8000);
		check(near(c.revolutions(), 0.0, 1e-12), "wraps backward across zero");
		// 1 周期 3000 カウントずつ 10 回 = 30000 カウント
		std::uint16_t a = 8000;
		for (int i = 0; i < 10; ++i) {
			a = static_cast<std::uint16_t>((a + 3000) % 8192);
			c.update(a);
		}
		check(near(c.revolutions(), 30000.0 / 8192.0, 1e-12), "counts many revolutions");
		c.reset();
		check(!c.valid(), "reset forgets the last angle");
	}

	void test_ramp() {
		Ramp r{0.0};
		check(r.arrived(), "starts at the goal");
		r.set_goal(0.1);
		// 0.2 m/s で 0.02 s ずつ: 0.004 m ずつ進む
		r.step(0.02, 0.2);
		check(near(r.position(), 0.004, 1e-12) && !r.arrived(), "moves at max_speed");
		for (int i = 0; i < 100; ++i) {
			r.step(0.02, 0.2);
		}
		check(r.position() == 0.1 && r.arrived(), "stops exactly at the goal");
		r.set_goal(0.0);
		r.step(0.02, 0.2);
		r.hold();
		check(r.arrived() && near(r.goal(), 0.096, 1e-12), "hold stops at the current command");
		r.step(0.02, 0.2);
		check(near(r.position(), 0.096, 1e-12), "does not move after hold");
	}
} // namespace

auto main() -> int {
	test_parse_feedback();
	test_rotor_counter();
	test_ramp();
	if (failures == 0) {
		std::printf("all passed\n");
	}
	return failures == 0 ? 0 : 1;
}
