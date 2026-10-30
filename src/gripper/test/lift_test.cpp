/// @file lift_test.cpp
/// 昇降の、ROS に依存しない部分のテスト。

#include <cmath>
#include <cstdio>
#include <source_location>
#include <limits>

#include "gripper/lift.hpp"
#include "gripper/current_guard.hpp"

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
		check(c.revolutions() == 0.0, "reset starts the count from 0 again (~/zero_lift)");
		c.update(5000);
		check(c.revolutions() == 0.0, "the first angle after a reset is the origin");
	}

	void test_current_guard() {
		using namespace std::chrono_literals;
		const auto t = CurrentGuard::TimePoint{} + 1s;
		CurrentGuard guard{8.0, 0.2, 0.1};
		check(near(current_amps(8000, false), 8.0, 1e-12), "C610 current is amperes");
		check(near(current_amps(-8192, true), -10.0, 1e-12), "C620 current is amperes");
		check(current_amps(-32768, false) < 0.0, "negative raw minimum does not overflow");
		check(guard.sample(9, 1, 0.15, t) == 0, "current spike does not trip");
		check(guard.sample(7, 1, 0.15, t + 50ms) == 0, "low current resets duration");
		for (int i = 0; i < 4; ++i) {
			check(guard.sample(-8, 1, 0.15, t + 100ms + i * 50ms) == 0, "continuous overload waits full duration");
		}
		check(guard.sample(-8, 1, 0.15, t + 300ms) == 1, "threshold equality trips at duration boundary");
		check(!guard.permits(0.18) && !guard.permits(0.15) && guard.permits(0.09), "upper stop rejects repeated target but permits retreat");
		guard.reset_timer();
		guard.sample(0, 0, 0.1, t + 350ms);
		check(!guard.permits(0.18), "low current, idle, and enable timer reset do not clear learned limit");
		for (int i = 0; i <= 4; ++i) { guard.sample(8, -1, 0.02, t + 400ms + i * 50ms); }
		check(!guard.permits(0) && guard.permits(0.09), "lower stop still allows movement between limits");
		guard.shift_origin(0.02);
		check(near(*guard.upper(), 0.13, 1e-12) && near(*guard.lower(), 0.0, 1e-12), "zeroing translates both physical limits");

		CurrentGuard gaps{8, 0.2, 0.1};
		gaps.sample(9, 1, 0.1, t);
		check(gaps.sample(9, 1, 0.1, t + 500ms) == 0, "feedback gap cannot count as overload");
		gaps.sample(9, 1, 0.1, t + 550ms);
		check(gaps.sample(9, -1, 0.1, t + 600ms) == 0, "reversal resets duration");
		gaps.sample(9, -1, 0.1, t + 650ms);
		check(gaps.sample(9, -1, 0.1, t + 400ms) == 0, "backward clock resets duration");
		gaps.sample(std::numeric_limits<double>::quiet_NaN(), -1, 0.1, t + 450ms);
		check(!gaps.upper() && !gaps.lower(), "discontinuous input cannot learn a limit");
		check(!gaps.permits(std::numeric_limits<double>::quiet_NaN()), "nonfinite goals rejected");
		for (const double bad : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
			bool rejected = false;
			try { CurrentGuard invalid{bad, 0.2, 0.1}; } catch (const std::invalid_argument&) { rejected = true; }
			check(rejected, "invalid current threshold rejected");
			rejected = false;
			try { CurrentGuard invalid{8.0, bad, 0.1}; } catch (const std::invalid_argument&) { rejected = true; }
			check(rejected, "invalid overload duration rejected");
		}
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
	test_current_guard();
	if (failures == 0) {
		std::printf("all passed\n");
	}
	return failures == 0 ? 0 : 1;
}
