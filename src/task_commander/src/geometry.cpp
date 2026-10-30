#include "task_commander/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace task_commander {
	auto pose_for_tool(const double target_x, const double target_y, const double yaw, const double offset_x, const double offset_y) -> Pose2 {
		const double c = std::cos(yaw);
		const double s = std::sin(yaw);
		return Pose2{
			.x = target_x - (c * offset_x - s * offset_y),
			.y = target_y - (s * offset_x + c * offset_y),
			.yaw = yaw,
		};
	}

	auto offset_pose(const Pose2& pose, const double dx, const double dy) -> Pose2 {
		const double c = std::cos(pose.yaw);
		const double s = std::sin(pose.yaw);
		return Pose2{.x = pose.x + c * dx - s * dy, .y = pose.y + s * dx + c * dy, .yaw = pose.yaw};
	}

	namespace {
		constexpr double outline_spacing = 0.01; ///< 外形の辺を見る間隔 [m]

		/// 点と箱の符号付き距離 (中なら負: 一番近い面までの距離)
		auto signed_distance(const Box& o, const double x, const double y) -> double {
			const double dx = std::max({o.xmin - x, 0.0, x - o.xmax});
			const double dy = std::max({o.ymin - y, 0.0, y - o.ymax});
			if (dx == 0.0 && dy == 0.0) {
				return -std::min({x - o.xmin, o.xmax - x, y - o.ymin, o.ymax - y});
			}
			return std::hypot(dx, dy);
		}

		auto inside(const std::vector<Point>& polygon, const double x, const double y) -> bool {
			bool in = false;
			for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
				const auto& a = polygon[i];
				const auto& b = polygon[j];
				if ((a.y > y) != (b.y > y) && x < a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y)) {
					in = !in;
				}
			}
			return in;
		}

		auto to_field(const Pose2& pose, const Point& p) -> Point {
			const double c = std::cos(pose.yaw);
			const double s = std::sin(pose.yaw);
			return Point{.x = pose.x + c * p.x - s * p.y, .y = pose.y + s * p.x + c * p.y};
		}

		auto wrap(const double a) -> double { return std::remainder(a, 2.0 * std::numbers::pi); }
	} // namespace

	auto clearance(const Field& field, const Pose2& pose, const Footprint& footprint) -> double {
		std::vector<Point> polygon{};
		for (const auto& p : footprint) {
			polygon.push_back(to_field(pose, p));
		}
		const auto& b = field.bounds;
		double gap = std::numeric_limits<double>::infinity();
		// 外周は凸なので、頂点だけ見ればよい
		for (const auto& p : polygon) {
			gap = std::min({gap, p.x - b.xmin, b.xmax - p.x, p.y - b.ymin, b.ymax - p.y});
		}
		// 障害物 (箱) は、外形の辺の点との距離と、外形の中に入った箱の角を見る
		for (std::size_t i = 0; i < polygon.size(); ++i) {
			const auto& a = polygon[i];
			const auto& c = polygon[(i + 1) % polygon.size()];
			const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(c.x - a.x, c.y - a.y) / outline_spacing)));
			for (int k = 0; k < n; ++k) {
				const double t = static_cast<double>(k) / n;
				const double x = a.x + t * (c.x - a.x);
				const double y = a.y + t * (c.y - a.y);
				for (const auto& o : field.obstacles) {
					gap = std::min(gap, signed_distance(o, x, y));
				}
			}
		}
		for (const auto& o : field.obstacles) {
			for (const auto& [x, y] : {std::pair{o.xmin, o.ymin}, {o.xmax, o.ymin}, {o.xmax, o.ymax}, {o.xmin, o.ymax}}) {
				if (inside(polygon, x, y)) {
					gap = std::min(gap, -outline_spacing);
				}
			}
		}
		return gap;
	}

	auto sweep_clearance(const Field& field, const std::vector<Pose2>& poses, const Footprint& footprint, const double step) -> SweepCheck {
		SweepCheck worst{.clearance = std::numeric_limits<double>::infinity()};
		const auto look = [&](const Pose2& p) {
			const double c = clearance(field, p, footprint);
			if (c < worst.clearance) {
				worst = SweepCheck{.clearance = c, .x = p.x, .y = p.y, .yaw = p.yaw};
			}
		};
		if (poses.empty()) {
			return worst;
		}
		// 回るときに一番大きく動く点 (中心から一番遠い頂点) が step より動かないように刻む
		double reach = 0.0;
		for (const auto& p : footprint) {
			reach = std::max(reach, std::hypot(p.x, p.y));
		}
		look(poses.front());
		for (std::size_t i = 1; i < poses.size(); ++i) {
			const auto& a = poses[i - 1];
			const auto& b = poses[i];
			const double turn = wrap(b.yaw - a.yaw);
			const double travel = std::hypot(b.x - a.x, b.y - a.y) + std::abs(turn) * reach;
			const int n = std::max(1, static_cast<int>(std::ceil(travel / step)));
			for (int k = 1; k <= n; ++k) {
				const double t = static_cast<double>(k) / n;
				look(Pose2{.x = a.x + t * (b.x - a.x), .y = a.y + t * (b.y - a.y), .yaw = a.yaw + t * turn});
			}
		}
		return worst;
	}

	auto footprint_from_flat(const std::vector<double>& flat, const std::string& what) -> Footprint {
		if (flat.size() % 2 != 0 || flat.size() < 6) {
			throw std::invalid_argument(what + " must be [x0, y0, x1, y1, ...] with at least 3 points");
		}
		Footprint footprint{};
		for (std::size_t i = 0; i < flat.size(); i += 2) {
			footprint.push_back(Point{flat[i], flat[i + 1]});
		}
		return footprint;
	}

	auto boxes_from_flat(const std::vector<double>& flat, const std::string& what) -> std::vector<Box> {
		if (flat.size() % 4 != 0) {
			throw std::invalid_argument(what + " must be [xmin, ymin, xmax, ymax, ...] (a multiple of 4 numbers)");
		}
		std::vector<Box> boxes{};
		for (std::size_t i = 0; i < flat.size(); i += 4) {
			boxes.push_back(Box{flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
		}
		return boxes;
	}
} // namespace task_commander
