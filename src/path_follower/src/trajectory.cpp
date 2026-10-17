/// @file trajectory.cpp
#include "path_follower/trajectory.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <istream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_follower {
	namespace {
		/// 連続する3点を通る円の曲率 (向きは見ない)
		auto curvature(const PathPoint& a, const PathPoint& b, const PathPoint& c) -> double {
			const double ab = std::hypot(b.x - a.x, b.y - a.y);
			const double bc = std::hypot(c.x - b.x, c.y - b.y);
			const double ca = std::hypot(a.x - c.x, a.y - c.y);
			const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
			const double denom = ab * bc * ca;
			return denom > 0.0 ? 2.0 * std::abs(cross) / denom : 0.0;
		}
	}

	Trajectory::Trajectory(std::vector<PathPoint> points, const Limits& limits) : points_{std::move(points)} {
		if (this->points_.size() < 2) { throw std::invalid_argument("a path needs at least two points"); }
		if (!(limits.max_speed > 0.0 && limits.max_accel > 0.0 && limits.max_lateral_accel > 0.0 && limits.max_yaw_rate > 0.0)) {
			throw std::invalid_argument("every limit must be positive");
		}
		const std::size_t n = this->points_.size();

		this->s_.assign(n, 0.0);
		for (std::size_t i = 1; i < n; ++i) {
			const double ds = std::hypot(this->points_[i].x - this->points_[i - 1].x, this->points_[i].y - this->points_[i - 1].y);
			if (!(ds > 1e-9)) {
				throw std::invalid_argument("consecutive path points must differ in position (turning on the spot is not supported)");
			}
			this->s_[i] = this->s_[i - 1] + ds;
		}

		// 各点で許される速さ: 最高速、横加速度 (曲率)、角速度 (道のりあたりの yaw の変化)
		std::vector<double> cap(n, limits.max_speed);
		for (std::size_t i = 0; i < n; ++i) {
			if (i > 0 && i + 1 < n) {
				const double k = curvature(this->points_[i - 1], this->points_[i], this->points_[i + 1]);
				if (k > 0.0) { cap[i] = std::min(cap[i], std::sqrt(limits.max_lateral_accel / k)); }
			}
			const std::size_t a = i > 0 ? i - 1 : i;
			const std::size_t b = i + 1 < n ? i + 1 : i;
			const double yaw_per_metre = std::abs(this->points_[b].yaw - this->points_[a].yaw) / (this->s_[b] - this->s_[a]);
			if (yaw_per_metre > 0.0) { cap[i] = std::min(cap[i], limits.max_yaw_rate / yaw_per_metre); }
		}

		// 両端で止まる。前から加速、後ろから減速の上限をかける
		this->speed_ = cap;
		this->speed_.front() = 0.0;
		this->speed_.back() = 0.0;
		for (std::size_t i = 1; i < n; ++i) {
			const double reachable = std::sqrt(this->speed_[i - 1] * this->speed_[i - 1] + 2.0 * limits.max_accel * (this->s_[i] - this->s_[i - 1]));
			this->speed_[i] = std::min(this->speed_[i], reachable);
		}
		for (std::size_t i = n - 1; i-- > 0;) {
			const double reachable = std::sqrt(this->speed_[i + 1] * this->speed_[i + 1] + 2.0 * limits.max_accel * (this->s_[i + 1] - this->s_[i]));
			this->speed_[i] = std::min(this->speed_[i], reachable);
		}

		// 区間ごとに等加速度とみなして時刻を付ける
		this->time_.assign(n, 0.0);
		for (std::size_t i = 1; i < n; ++i) {
			const double mean = 0.5 * (this->speed_[i - 1] + this->speed_[i]);
			this->time_[i] = this->time_[i - 1] + (this->s_[i] - this->s_[i - 1]) / mean;
		}
	}

	auto Trajectory::sample(const double t) const -> State {
		const auto& p = this->points_;
		if (!(t < this->duration())) {
			return State{.x = p.back().x, .y = p.back().y, .yaw = p.back().yaw, .s = this->length(), .finished = true};
		}
		const double tc = std::max(t, 0.0);
		// time_[i] <= tc < time_[i + 1]
		const auto upper = std::upper_bound(this->time_.begin(), this->time_.end(), tc);
		const auto i = static_cast<std::size_t>(std::max<std::ptrdiff_t>(1, upper - this->time_.begin())) - 1;

		const double ds = this->s_[i + 1] - this->s_[i];
		const double v0 = this->speed_[i];
		const double v1 = this->speed_[i + 1];
		const double accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
		const double tau = tc - this->time_[i];
		const double along = std::clamp(v0 * tau + 0.5 * accel * tau * tau, 0.0, ds);
		const double u = along / ds;
		const double v = std::max(0.0, v0 + accel * tau);

		const double dx = (p[i + 1].x - p[i].x) / ds;
		const double dy = (p[i + 1].y - p[i].y) / ds;
		const double dyaw = (p[i + 1].yaw - p[i].yaw) / ds;
		return State{
			.x = p[i].x + u * (p[i + 1].x - p[i].x),
			.y = p[i].y + u * (p[i + 1].y - p[i].y),
			.yaw = p[i].yaw + u * (p[i + 1].yaw - p[i].yaw),
			.vx = v * dx,
			.vy = v * dy,
			.omega = v * dyaw,
			.s = this->s_[i] + along,
			.finished = false,
		};
	}

	auto polyline_path(const std::vector<PathPoint>& vertices, const double spacing) -> std::vector<PathPoint> {
		if (!(spacing > 0.0)) { throw std::invalid_argument("spacing must be positive"); }
		std::vector<PathPoint> points{};
		if (vertices.empty()) { throw std::invalid_argument("a path needs at least two points"); }
		PathPoint last = vertices.front();
		points.push_back(last);
		for (std::size_t i = 1; i < vertices.size(); ++i) {
			PathPoint next = vertices[i];
			// 前の頂点から近いほうへ回る
			next.yaw = last.yaw + std::remainder(next.yaw - last.yaw, 2.0 * std::numbers::pi);
			const double length = std::hypot(next.x - last.x, next.y - last.y);
			if (!(length > 1e-6)) {
				if (std::abs(next.yaw - last.yaw) > 1e-6) {
					throw std::invalid_argument("vertices at the same position with different yaw (turning on the spot is not supported)");
				}
				continue;
			}
			const auto n = static_cast<std::size_t>(std::ceil(length / spacing));
			for (std::size_t k = 1; k <= n; ++k) {
				const double t = static_cast<double>(k) / static_cast<double>(n);
				points.push_back(PathPoint{
					.x = last.x + t * (next.x - last.x),
					.y = last.y + t * (next.y - last.y),
					.yaw = last.yaw + t * (next.yaw - last.yaw),
				});
			}
			last = next;
		}
		if (points.size() < 2) { throw std::invalid_argument("a path needs at least two points at different positions"); }
		return points;
	}

	auto load_path_csv(std::istream& in) -> std::vector<PathPoint> {
		std::vector<PathPoint> points{};
		std::string line{};
		std::size_t line_number = 0;
		while (std::getline(in, line)) {
			++line_number;
			const auto first = line.find_first_not_of(" \t\r");
			if (first == std::string::npos || line[first] == '#') { continue; }
			std::replace(line.begin(), line.end(), ',', ' ');
			std::istringstream row{line};
			PathPoint p{};
			std::string rest{};
			if (!(row >> p.x >> p.y >> p.yaw) || (row >> rest)) {
				// 数値の行がまだ無ければ見出しとみなす
				if (points.empty() && !std::isdigit(static_cast<unsigned char>(line[first])) && line[first] != '-' && line[first] != '+' && line[first] != '.') {
					continue;
				}
				throw std::invalid_argument("path file line " + std::to_string(line_number) + " is not 'x, y, yaw'");
			}
			points.push_back(p);
		}
		return points;
	}
}
