#include "task_commander/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

	auto clearance(const Field& field, const double x, const double y, const double radius) -> double {
		const auto& b = field.bounds;
		double gap = std::min({x - b.xmin, b.xmax - x, y - b.ymin, b.ymax - y}) - radius;
		for (const auto& o : field.obstacles) {
			const double dx = std::max({o.xmin - x, 0.0, x - o.xmax});
			const double dy = std::max({o.ymin - y, 0.0, y - o.ymax});
			gap = std::min(gap, std::hypot(dx, dy) - radius);
		}
		return gap;
	}

	auto sweep_clearance(const Field& field, const std::vector<Pose2>& poses, const double radius, const double step) -> SweepCheck {
		SweepCheck worst{.clearance = std::numeric_limits<double>::infinity()};
		const auto look = [&](const double x, const double y) {
			const double c = clearance(field, x, y, radius);
			if (c < worst.clearance) {
				worst = SweepCheck{.clearance = c, .x = x, .y = y};
			}
		};
		if (poses.empty()) {
			return worst;
		}
		look(poses.front().x, poses.front().y);
		for (std::size_t i = 1; i < poses.size(); ++i) {
			const auto& a = poses[i - 1];
			const auto& b = poses[i];
			const double length = std::hypot(b.x - a.x, b.y - a.y);
			const int n = std::max(1, static_cast<int>(std::ceil(length / step)));
			for (int k = 1; k <= n; ++k) {
				const double t = static_cast<double>(k) / n;
				look(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
			}
		}
		return worst;
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
