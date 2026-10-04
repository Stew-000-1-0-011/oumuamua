#include "oumuamua_sim/plant.hpp"

#include <cmath>
#include <numbers>

namespace oumuamua_sim {
	namespace {
		/// 1 次遅れの 1 ステップ (厳密な離散化)
		auto lag(const double current, const double target, const double tau, const double dt) -> double {
			if (!(tau > 0.0)) {
				return target;
			}
			return target + (current - target) * std::exp(-dt / tau);
		}
	}

	auto wrap_angle(const double a) -> double {
		constexpr double two_pi = 2.0 * std::numbers::pi;
		double r = std::fmod(a + std::numbers::pi, two_pi);
		if (r < 0.0) {
			r += two_pi;
		}
		return r - std::numbers::pi;
	}

	void Plant::step(const Twist2& cmd, const double dt) {
		if (!(dt > 0.0)) {
			return;
		}
		const auto& p = this->params_;

		// 実際に出る速度の目標 (滑りなどで指令からずれる)
		const Twist2 target{
			.vx = cmd.vx * p.velocity_scale_linear,
			.vy = cmd.vy * p.velocity_scale_linear,
			.omega = cmd.omega * p.velocity_scale_angular,
		};

		// 1 次遅れで追従
		Twist2 next{
			.vx = lag(this->velocity_.vx, target.vx, p.tau_linear, dt),
			.vy = lag(this->velocity_.vy, target.vy, p.tau_linear, dt),
			.omega = lag(this->velocity_.omega, target.omega, p.tau_angular, dt),
		};

		// 加速度制限 (並進は向きを保ったまま大きさで縮める)
		if (p.max_accel_linear > 0.0) {
			const double dvx = next.vx - this->velocity_.vx;
			const double dvy = next.vy - this->velocity_.vy;
			const double dv = std::hypot(dvx, dvy);
			const double allowed = p.max_accel_linear * dt;
			if (dv > allowed) {
				next.vx = this->velocity_.vx + dvx * allowed / dv;
				next.vy = this->velocity_.vy + dvy * allowed / dv;
			}
		}
		if (p.max_accel_angular > 0.0) {
			const double dw = next.omega - this->velocity_.omega;
			const double allowed = p.max_accel_angular * dt;
			if (std::abs(dw) > allowed) {
				next.omega = this->velocity_.omega + std::copysign(allowed, dw);
			}
		}

		// ステップの中点の向きで機体座標系 -> フィールド座標系に回して積分する
		const double mid_yaw = this->pose_.yaw + 0.5 * (this->velocity_.omega + next.omega) * 0.5 * dt;
		const double vx = 0.5 * (this->velocity_.vx + next.vx);
		const double vy = 0.5 * (this->velocity_.vy + next.vy);
		const double c = std::cos(mid_yaw);
		const double s = std::sin(mid_yaw);
		this->pose_.x += (c * vx - s * vy) * dt;
		this->pose_.y += (s * vx + c * vy) * dt;
		this->pose_.yaw = wrap_angle(this->pose_.yaw + 0.5 * (this->velocity_.omega + next.omega) * dt);
		this->velocity_ = next;
	}
}
