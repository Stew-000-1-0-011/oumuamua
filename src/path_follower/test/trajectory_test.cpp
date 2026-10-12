/// @file trajectory_test.cpp
/// 速度計画のテスト (ROS不要)。

#include <cmath>
#include <cstdio>
#include <numbers>
#include <source_location>
#include <sstream>
#include <stdexcept>

#include "path_follower/trajectory.hpp"

namespace {
	using namespace path_follower;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol) -> bool { return std::abs(a - b) <= tol; }

	/// x 軸に沿った直線。yaw は始点 0 から終点 yaw_end まで一定の割合で回す
	auto line(const double length, const double yaw_end = 0.0, const double step = 0.01) -> std::vector<PathPoint> {
		std::vector<PathPoint> p{};
		const int n = static_cast<int>(std::round(length / step));
		for (int i = 0; i <= n; ++i) {
			const double u = static_cast<double>(i) / n;
			p.push_back({u * length, 0.0, u * yaw_end});
		}
		return p;
	}

	void test_trapezoid_on_a_long_line() {
		// 加速 1 s (0.5 m) + 等速 1 s (1 m) + 減速 1 s (0.5 m)
		const Trajectory t{line(2.0), Limits{.max_speed = 1.0, .max_accel = 1.0}};
		check(near(t.length(), 2.0, 1e-9), "length is the sum of the segments");
		check(near(t.duration(), 3.0, 0.02), "a 2 m line at 1 m/s, 1 m/s^2 takes 3 s");
		const auto start = t.sample(0.0);
		check(near(start.vx, 0.0, 1e-9) && near(start.x, 0.0, 1e-9), "starts at rest at the first point");
		const auto mid = t.sample(1.5);
		check(near(mid.x, 1.0, 0.02) && near(mid.vx, 1.0, 0.02) && near(mid.vy, 0.0, 1e-9), "cruises at max speed in the middle");
		const auto accel = t.sample(0.5);
		check(near(accel.vx, 0.5, 0.03) && near(accel.x, 0.125, 0.02), "accelerates at max_accel");
	}

	void test_triangle_on_a_short_line() {
		// 0.5 m では最高速に届かない: 頂点の速さ sqrt(a L) = 0.707、所要 2 * 0.707
		const Trajectory t{line(0.5), Limits{.max_speed = 1.0, .max_accel = 1.0}};
		check(near(t.duration(), 2.0 * std::sqrt(0.5), 0.03), "a short line never reaches max speed");
		double peak = 0.0;
		for (double x = 0.0; x <= t.duration(); x += 0.01) { peak = std::max(peak, t.sample(x).vx); }
		check(near(peak, std::sqrt(0.5), 0.03), "peak speed is sqrt(a L)");
	}

	void test_ends_at_rest_and_stays() {
		const Trajectory t{line(1.0), Limits{}};
		const auto end = t.sample(t.duration());
		check(end.finished && near(end.x, 1.0, 1e-9) && near(end.vx, 0.0, 1e-9), "stops exactly at the last point");
		const auto later = t.sample(t.duration() + 10.0);
		check(later.finished && near(later.x, 1.0, 1e-9) && near(later.vx, 0.0, 1e-9), "stays there afterwards");
		check(!t.sample(0.5 * t.duration()).finished, "not finished halfway");
		check(near(t.sample(-1.0).x, 0.0, 1e-9), "negative time is the start");
	}

	void test_slows_down_in_a_curve() {
		// 直線 1 m -> 半径 0.3 m の四分円 -> 直線 1 m。横加速度 1 m/s^2 なら円弧では sqrt(0.3) = 0.548 m/s まで
		std::vector<PathPoint> p{};
		for (int i = 0; i < 100; ++i) { p.push_back({i * 0.01, 0.0, 0.0}); }
		const double r = 0.3;
		for (int i = 0; i <= 47; ++i) {
			const double a = (std::numbers::pi / 2) * i / 47;
			p.push_back({1.0 + r * std::sin(a), r - r * std::cos(a), 0.0});
		}
		for (int i = 1; i <= 100; ++i) { p.push_back({1.0 + r, r + i * 0.01, 0.0}); }
		const Trajectory t{p, Limits{.max_speed = 2.0, .max_accel = 2.0, .max_lateral_accel = 1.0}};

		double fastest_in_arc = 0.0;
		double fastest_overall = 0.0;
		for (double x = 0.0; x <= t.duration(); x += 0.005) {
			const auto s = t.sample(x);
			const double v = std::hypot(s.vx, s.vy);
			fastest_overall = std::max(fastest_overall, v);
			if (s.s > 1.02 && s.s < 1.0 + r * std::numbers::pi / 2 - 0.02) { fastest_in_arc = std::max(fastest_in_arc, v); }
		}
		check(fastest_in_arc <= std::sqrt(r) * 1.05, "lateral acceleration limits the speed in the arc");
		check(fastest_in_arc >= std::sqrt(r) * 0.85, "but it does not crawl through the arc");
		check(fastest_overall > 1.0, "and it is faster on the straights");
		const auto in_arc = t.sample(t.duration() / 2);
		check(in_arc.vx > 0.05 && in_arc.vy > 0.05, "velocity follows the tangent in the arc");
	}

	void test_yaw_rate_limits_the_speed() {
		// 1 m 進む間に 90 度回る。角速度 0.5 rad/s なら 0.5 / (pi/2) = 0.318 m/s まで
		const Trajectory t{line(1.0, std::numbers::pi / 2), Limits{.max_speed = 2.0, .max_accel = 5.0, .max_yaw_rate = 0.5}};
		double fastest = 0.0;
		double fastest_turn = 0.0;
		for (double x = 0.0; x <= t.duration(); x += 0.005) {
			const auto s = t.sample(x);
			fastest = std::max(fastest, s.vx);
			fastest_turn = std::max(fastest_turn, std::abs(s.omega));
		}
		check(near(fastest, 0.5 / (std::numbers::pi / 2), 0.02), "yaw rate caps the speed along the path");
		check(fastest_turn <= 0.5 * 1.02 && fastest_turn >= 0.45, "omega stays at the limit");
		check(near(t.sample(t.duration()).yaw, std::numbers::pi / 2, 1e-9), "ends at the last yaw");
	}

	void test_progress_is_monotonic() {
		const Trajectory t{line(1.5), Limits{}};
		double last = -1.0;
		bool ok = true;
		for (double x = 0.0; x <= t.duration() + 0.1; x += 0.01) {
			const double s = t.sample(x).s;
			ok = ok && s >= last;
			last = s;
		}
		check(ok, "distance along the path never goes back");
	}

	void test_rejects_bad_input() {
		const auto throws = [](auto&& f) {
			try {
				f();
			} catch (const std::invalid_argument&) { return true; }
			return false;
		};
		check(throws([] { const Trajectory t{{PathPoint{0, 0, 0}}, Limits{}}; }), "one point is not a path");
		check(throws([] { const Trajectory t{{PathPoint{0, 0, 0}, PathPoint{0, 0, 1}}, Limits{}}; }), "turning on the spot is rejected");
		check(throws([] { const Trajectory t{line(1.0), Limits{.max_speed = 0.0}}; }), "limits must be positive");
	}

	void test_load_csv() {
		std::istringstream in{"# comment\nx,y,yaw\n0.0, 0.0, 0.0\n\n1.0,0.5,-1.57\n"};
		const auto p = load_path_csv(in);
		check(p.size() == 2 && near(p[1].x, 1.0, 1e-12) && near(p[1].y, 0.5, 1e-12) && near(p[1].yaw, -1.57, 1e-12), "reads x, y, yaw rows and skips the header and comments");
		std::istringstream bad{"0,0,0\n1,oops,0\n"};
		bool threw = false;
		try {
			load_path_csv(bad);
		} catch (const std::invalid_argument&) { threw = true; }
		check(threw, "a row that is not three numbers is an error");
	}
}

auto main() -> int {
	test_trapezoid_on_a_long_line();
	test_triangle_on_a_short_line();
	test_ends_at_rest_and_stays();
	test_slows_down_in_a_curve();
	test_yaw_rate_limits_the_speed();
	test_progress_is_monotonic();
	test_rejects_bad_input();
	test_load_csv();
	if (failures == 0) { std::printf("all tests passed\n"); }
	return failures == 0 ? 0 : 1;
}
