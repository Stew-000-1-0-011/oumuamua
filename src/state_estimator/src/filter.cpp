#include "state_estimator/filter.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Eigen/Cholesky>

namespace state_estimator {
	auto wrap_angle(const double a) -> double {
		constexpr double two_pi = 2.0 * std::numbers::pi;
		double r = std::fmod(a + std::numbers::pi, two_pi);
		if (r < 0.0) {
			r += two_pi;
		}
		return r - std::numbers::pi;
	}

	void Filter::initialize(const double t, const Vec3& pose, const Mat3& pose_cov) {
		State s{};
		s.stamp = t;
		s.x.head<3>() = pose;
		s.x(2) = wrap_angle(pose(2));
		s.x.tail<3>() = this->input_at(t);
		s.P.setZero();
		s.P.topLeftCorner<3, 3>() = pose_cov;
		const auto& p = this->params_;
		s.P(3, 3) = s.P(4, 4) = p.input_sigma_linear * p.input_sigma_linear + 1e-6;
		s.P(5, 5) = p.input_sigma_angular * p.input_sigma_angular + 1e-6;
		this->anchor_ = s;
		this->prune_inputs();
	}

	void Filter::add_input(const double t, const Vec3& body_velocity) {
		// 時刻順に入れる
		auto it = this->inputs_.end();
		while (it != this->inputs_.begin() && std::prev(it)->t > t) {
			--it;
		}
		this->inputs_.insert(it, Input{t, body_velocity});
	}

	auto Filter::input_at(const double t) const -> Vec3 {
		Vec3 u = Vec3::Zero();
		for (const auto& in : this->inputs_) {
			if (in.t > t) {
				break;
			}
			u = in.u;
		}
		return u;
	}

	void Filter::prune_inputs() {
		if (!this->anchor_) {
			return;
		}
		// アンカーの時刻で効いている入力 (アンカー以前で最新) は残す
		while (this->inputs_.size() > 1 && this->inputs_[1].t <= this->anchor_->stamp) {
			this->inputs_.pop_front();
		}
	}

	void Filter::step(State& s, const Vec3& u, const double dt) const {
		const auto& p = this->params_;
		const Vec3 v0 = s.x.tail<3>();

		// 速度: 1 次遅れ (tau > 0) か、入力そのもの (tau = 0)
		Vec3 a{};  // v1 = u + (v0 - u) * a
		const double taus[3]{p.tau_linear, p.tau_linear, p.tau_angular};
		for (int i = 0; i < 3; ++i) {
			a(i) = taus[i] > 0.0 ? std::exp(-dt / taus[i]) : 0.0;
		}
		const Vec3 v1 = u + (v0 - u).cwiseProduct(a);

		// 姿勢: 区間の平均の速度を、区間の中点の向きで回して積分する。
		// tau = 0 の成分は区間の初めから入力の速度なので、平均ではなく入力そのもの
		Vec3 vm{};
		Vec3 dvm_dv0_diag{};
		for (int i = 0; i < 3; ++i) {
			if (taus[i] > 0.0) {
				vm(i) = 0.5 * (v0(i) + v1(i));
				dvm_dv0_diag(i) = 0.5 * (1.0 + a(i));
			} else {
				vm(i) = u(i);
				dvm_dv0_diag(i) = 0.0;
			}
		}
		const double yaw_mid = s.x(2) + 0.5 * vm(2) * dt;
		const double c = std::cos(yaw_mid);
		const double sn = std::sin(yaw_mid);

		Vec6 x1 = s.x;
		x1(0) += (c * vm(0) - sn * vm(1)) * dt;
		x1(1) += (sn * vm(0) + c * vm(1)) * dt;
		x1(2) = wrap_angle(s.x(2) + vm(2) * dt);
		x1.tail<3>() = v1;

		// ヤコビアン
		Mat6 F = Mat6::Identity();
		// d(位置)/d(yaw)
		F(0, 2) = (-sn * vm(0) - c * vm(1)) * dt;
		F(1, 2) = (c * vm(0) - sn * vm(1)) * dt;
		// d(姿勢)/d(v0)
		Eigen::Matrix<double, 3, 3> R2;
		R2 << c, -sn, 0.0, sn, c, 0.0, 0.0, 0.0, 1.0;
		const Mat3 dvm_dv0 = dvm_dv0_diag.asDiagonal();
		F.block<3, 3>(0, 3) = R2 * dvm_dv0 * dt;
		F.block<3, 3>(3, 3) = a.asDiagonal();

		Mat6 Q = Mat6::Zero();
		Q(0, 0) = Q(1, 1) = p.pose_noise_linear * dt;
		Q(2, 2) = p.pose_noise_angular * dt;
		const double qv[3]{p.velocity_noise_linear, p.velocity_noise_linear, p.velocity_noise_angular};
		const double sigma_in[3]{p.input_sigma_linear, p.input_sigma_linear, p.input_sigma_angular};
		for (int i = 0; i < 3; ++i) {
			// tau > 0: 速度の雑音を積分。tau = 0: 速度は入力そのものなので、入力の不確かさに置き換える
			Q(3 + i, 3 + i) = taus[i] > 0.0 ? qv[i] * dt : 0.0;
		}

		s.P = F * s.P * F.transpose() + Q;
		for (int i = 0; i < 3; ++i) {
			if (!(taus[i] > 0.0)) {
				// 速度は入力で決まり、前の速度とは相関しない
				s.P.row(3 + i).setZero();
				s.P.col(3 + i).setZero();
				s.P(3 + i, 3 + i) = sigma_in[i] * sigma_in[i] + 1e-9;
			}
		}
		s.x = x1;
		s.stamp += dt;
	}

	auto Filter::predict(const double t) const -> std::optional<State> {
		if (!this->anchor_) {
			return std::nullopt;
		}
		State s = *this->anchor_;
		if (t <= s.stamp) {
			return s;
		}
		const double step_max = std::max(1e-4, this->params_.max_step);
		// 入力が切り替わる時刻で区切りながら進める
		auto it = this->inputs_.begin();
		Vec3 u = Vec3::Zero();
		while (it != this->inputs_.end() && it->t <= s.stamp) {
			u = it->u;
			++it;
		}
		while (s.stamp < t) {
			const double next_change = it != this->inputs_.end() ? it->t : t;
			const double seg_end = std::min(t, next_change);
			while (s.stamp < seg_end - 1e-12) {
				const double dt = std::min(step_max, seg_end - s.stamp);
				this->step(s, u, dt);
			}
			s.stamp = std::max(s.stamp, seg_end);
			if (it != this->inputs_.end() && it->t <= s.stamp) {
				u = it->u;
				++it;
			}
		}
		return s;
	}

	auto Filter::update(const double t, const Vec3& z, const Mat3& R) -> UpdateResult {
		if (!this->anchor_) {
			return UpdateResult::not_initialized;
		}
		if (t < this->anchor_->stamp) {
			return UpdateResult::out_of_order;
		}
		State s = *this->predict(t);

		Vec3 innovation = z - s.x.head<3>();
		innovation(2) = wrap_angle(innovation(2));
		const Mat3 S = s.P.topLeftCorner<3, 3>() + R;
		const Eigen::LDLT<Mat3> S_ldlt{S};

		if (this->params_.gate_sigma > 0.0) {
			const double d2 = innovation.dot(S_ldlt.solve(innovation));
			if (d2 > this->params_.gate_sigma * this->params_.gate_sigma) {
				return UpdateResult::gated;
			}
		}

		// K = P H^T S^-1, H = [I 0]
		const Eigen::Matrix<double, 6, 3> PHt = s.P.leftCols<3>();
		const Eigen::Matrix<double, 6, 3> K = S_ldlt.solve(PHt.transpose()).transpose();
		s.x += K * innovation;
		s.x(2) = wrap_angle(s.x(2));
		// Joseph 形式で対称性と正定値性を保つ
		Eigen::Matrix<double, 6, 6> I_KH = Mat6::Identity();
		I_KH.leftCols<3>() -= K;
		s.P = I_KH * s.P * I_KH.transpose() + K * R * K.transpose();
		s.P = 0.5 * (s.P + s.P.transpose());

		this->anchor_ = s;
		this->prune_inputs();
		return UpdateResult::accepted;
	}

	void Filter::inflate(const Mat3& pose_cov) {
		if (this->anchor_) {
			this->anchor_->P.topLeftCorner<3, 3>() += pose_cov;
		}
	}

	void Filter::advance_anchor(const double now) {
		if (!this->anchor_) {
			return;
		}
		const double horizon = this->params_.max_horizon;
		if (horizon > 0.0 && now - this->anchor_->stamp > horizon) {
			this->anchor_ = *this->predict(now - 0.5 * horizon);
			this->prune_inputs();
		}
	}
}
