#include "state_estimator/belief_geometry.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>

namespace state_estimator {
	namespace {
		/// 機体の 6 自由度 q = (x, y, z, roll, pitch, yaw) -> T_field_base
		auto base_from_params(const Vec6& q) -> SE3 {
			const Eigen::Matrix3d R = (Eigen::AngleAxisd(q(5), Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(q(4), Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(q(3), Eigen::Vector3d::UnitX()))
				.toRotationMatrix();
			return SE3{Sophus::SO3d{R}, Eigen::Vector3d{q(0), q(1), q(2)}};
		}

		/// 対称行列の固有値を下から丸めて作り直す
		auto clamp_eigen(const Mat6& A, const double min_eig) -> Mat6 {
			const Eigen::SelfAdjointEigenSolver<Mat6> es{0.5 * (A + A.transpose())};
			const Vec6 ev = es.eigenvalues().cwiseMax(min_eig);
			return es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
		}

		constexpr double h = 1e-6;
	}

	auto planar_to_se3(const Vec3& pose) -> SE3 {
		Vec6 q = Vec6::Zero();
		q(0) = pose(0);
		q(1) = pose(1);
		q(5) = pose(2);
		return base_from_params(q);
	}

	auto base_pose_from_field(const SE3& M_field, const SE3& T_base_lidar) -> Vec3 {
		const SE3 T_field_base = M_field.inverse() * T_base_lidar.inverse();
		const Eigen::Matrix3d R = T_field_base.rotationMatrix();
		return Vec3{T_field_base.translation().x(), T_field_base.translation().y(), std::atan2(R(1, 0), R(0, 0))};
	}

	auto make_prior(
		const Vec3& base_pose,
		const Mat3& base_cov,
		const SE3& T_base_lidar,
		const SE3& T_field_obj,
		const PriorNoise& noise
	) -> Belief {
		Vec6 q0 = Vec6::Zero();
		q0(0) = base_pose(0);
		q0(1) = base_pose(1);
		q0(5) = base_pose(2);
		const auto object_in_lidar = [&](const Vec6& q) {
			return (base_from_params(q) * T_base_lidar).inverse() * T_field_obj;
		};
		const SE3 mean = object_in_lidar(q0);

		// 機体の 6 自由度の微小変化 -> 物体の左摂動 xi (センサ座標系) のヤコビアン
		Mat6 J{};
		for (int k = 0; k < 6; ++k) {
			Vec6 dq = Vec6::Zero();
			dq(k) = h;
			const Vec6 plus = (object_in_lidar(q0 + dq) * mean.inverse()).log();
			const Vec6 minus = (object_in_lidar(q0 - dq) * mean.inverse()).log();
			J.col(k) = (plus - minus) / (2.0 * h);
		}

		// 機体の共分散: 平面 (x, y, yaw) は推定から、面外 (z, roll, pitch) は取付の精度から
		Mat6 Sq = Mat6::Zero();
		const int planar[3]{0, 1, 5};
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				Sq(planar[i], planar[j]) = base_cov(i, j);
			}
		}
		Sq(2, 2) = noise.sigma_z * noise.sigma_z;
		Sq(3, 3) = Sq(4, 4) = noise.sigma_tilt * noise.sigma_tilt;

		Mat6 S = J * Sq * J.transpose();
		// フィールドに対する物体の位置関係の不確かさ (向きによらないよう、xy と回転は等方に)
		const double rxy = noise.relation_sigma_xy * noise.relation_sigma_xy;
		const double ryaw = noise.relation_sigma_yaw * noise.relation_sigma_yaw;
		S(0, 0) += rxy;
		S(1, 1) += rxy;
		S(2, 2) += rxy;
		S(3, 3) += ryaw;
		S(4, 4) += ryaw;
		S(5, 5) += ryaw;

		// 数値誤差で特異にならないよう、ごく小さく下駄をはかせてから逆にする
		S = clamp_eigen(S, 1e-12);
		return Belief{mean, S.inverse()};
	}

	auto remove_prior(const Belief& posterior, const Belief& prior) -> Belief {
		// 事後の平均のまわりの局所座標で、事前の平均は xi_pri
		const Vec6 xi_pri = (prior.mean * posterior.mean.inverse()).log();
		// 事後 = 事前 + スキャン なので、スキャンの情報は差
		const Mat6 L_scan = clamp_eigen(posterior.information - prior.information, 0.0);
		// 事後の平均では勾配が 0: L_pri (0 - xi_pri) + L_scan (0 - xi_scan) = 0
		const Vec6 b = -prior.information * xi_pri;

		// 情報の無い方向は 0 にする (擬似逆)
		const Eigen::SelfAdjointEigenSolver<Mat6> es{L_scan};
		const double max_eig = std::max(es.eigenvalues().maxCoeff(), 0.0);
		Vec6 inv = Vec6::Zero();
		for (int i = 0; i < 6; ++i) {
			const double e = es.eigenvalues()(i);
			inv(i) = e > 1e-9 * max_eig && e > 0.0 ? 1.0 / e : 0.0;
		}
		const Vec6 xi_scan = es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose() * b;
		return Belief{SE3::exp(xi_scan) * posterior.mean, L_scan};
	}

	auto planar_measurement(const Belief& field_belief, const SE3& T_base_lidar, const Mat3& floor)
		-> PlanarMeasurement {
		// 機体の 6 自由度 q = (x, y, z, roll, pitch, yaw) を、この信念が指す姿勢のまわりで考える。
		// 2D LiDAR では面外 (z, roll, pitch) は見えないが、取付で決まっていて既知なので、
		// 周辺化 (不確かさとして足す) ではなく、既知として条件付ける。
		// LiDAR は床から離れているので、面外の傾きは「てこ」で水平位置に漏れる。周辺化すると
		// 見えない傾きの分散がそのまま水平位置の巨大な分散になってしまう。
		const Vec3 q0_planar = base_pose_from_field(field_belief.mean, T_base_lidar);
		Vec6 q0 = Vec6::Zero();
		q0(0) = q0_planar(0);
		q0(1) = q0_planar(1);
		q0(5) = q0_planar(2);
		const auto field_in_lidar = [&](const Vec6& q) { return (base_from_params(q) * T_base_lidar).inverse(); };
		const SE3 M0 = field_in_lidar(q0);

		// q -> 左摂動 xi のヤコビアン
		Mat6 J{};
		for (int k = 0; k < 6; ++k) {
			Vec6 dq = Vec6::Zero();
			dq(k) = h;
			J.col(k) = ((field_in_lidar(q0 + dq) * M0.inverse()).log() - (field_in_lidar(q0 - dq) * M0.inverse()).log())
				/ (2.0 * h);
		}

		// q の情報行列。面外を既知とするので、平面の 3 成分のブロックをそのまま使う
		const Mat6 L = clamp_eigen(field_belief.information, 0.0);
		const Mat6 Lq = J.transpose() * L * J;
		const int planar[3]{0, 1, 5};
		Mat3 Lp{};
		Vec3 gp{};
		// 信念の平均と q0 のずれ (面外の食い違いのぶん) を、平面の成分で吸収する
		const Vec6 xi0 = (field_belief.mean * M0.inverse()).log();
		const Vec6 gq = J.transpose() * L * xi0;
		for (int i = 0; i < 3; ++i) {
			gp(i) = gq(planar[i]);
			for (int j = 0; j < 3; ++j) {
				Lp(i, j) = Lq(planar[i], planar[j]);
			}
		}

		// 情報の無い方向は大きな分散にする (擬似逆。固有値の下限で切る)
		const Eigen::SelfAdjointEigenSolver<Mat3> es{0.5 * (Lp + Lp.transpose())};
		const double max_eig = std::max(es.eigenvalues().maxCoeff(), 0.0);
		Vec3 inv{};
		for (int i = 0; i < 3; ++i) {
			const double e = es.eigenvalues()(i);
			inv(i) = e > 1e-9 * max_eig && e > 0.0 ? 1.0 / e : 1e6;
		}
		const Mat3 cov = es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose();
		Vec3 dz = Vec3::Zero();
		for (int i = 0; i < 3; ++i) {
			const double e = es.eigenvalues()(i);
			if (e > 1e-9 * max_eig && e > 0.0) {
				dz += es.eigenvectors().col(i) * (es.eigenvectors().col(i).dot(gp) / e);
			}
		}
		Vec3 z = q0_planar + dz;
		z(2) = wrap_angle(z(2));
		Mat3 out = cov + floor;
		out = 0.5 * (out + out.transpose());
		return PlanarMeasurement{z, out};
	}
}
