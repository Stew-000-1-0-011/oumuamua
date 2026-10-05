#pragma once
/// @file belief_geometry.hpp
/// 機体の平面姿勢と、sotoba の信念 (LiDAR から見た物体の姿勢と情報行列) の相互変換。ROS非依存。
///
/// sotoba の約束:
/// - 物体の姿勢 M は「物体ローカル座標系を LiDAR 座標系へ写す SE3」(= LiDAR から見た物体)
/// - 情報行列は 6x6、成分順序は [並進3, 回転3]、摂動は左 (センサ座標系): M = exp(xi) * mean
///
/// 機体の姿勢は T_field_base。LiDAR の取付は T_base_lidar。
/// フィールド座標系での物体の姿勢を T_field_obj とすると M = (T_field_base * T_base_lidar)^-1 * T_field_obj。

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "state_estimator/filter.hpp"

namespace state_estimator {
	using SE3 = Sophus::SE3d;

	/// 平面の姿勢 (x, y, yaw) -> SE3 (z = 0、水平)
	auto planar_to_se3(const Vec3& pose) -> SE3;

	/// 事前分布を作るときの、平面の推定に含まれない不確かさ
	struct PriorNoise {
		/// 機体の面外 (z, roll, pitch) の標準偏差。取付で決まっていて動かないので小さく
		double sigma_z{0.005};
		double sigma_tilt{0.005};
		/// フィールドに対する物体の位置関係の不確かさ (センサ座標系、フィールド以外の物体に足す)
		double relation_sigma_xy{0.0};
		double relation_sigma_yaw{0.0};
	};

	struct Belief {
		SE3 mean{};
		Mat6 information{Mat6::Zero()};
	};

	/// 機体の平面姿勢とその共分散から、物体の事前分布を作る
	auto make_prior(
		const Vec3& base_pose,
		const Mat3& base_cov,
		const SE3& T_base_lidar,
		const SE3& T_field_obj,
		const PriorNoise& noise
	) -> Belief;

	/// 事後分布から、自分が送った事前分布の分を差し引いて「スキャンだけ」の信念にする。
	/// 情報行列の負の固有値は 0 に丸める (情報の無い方向)
	auto remove_prior(const Belief& posterior, const Belief& prior) -> Belief;

	/// フィールド物体の信念から、機体の平面姿勢の観測とその共分散を作る。
	/// 情報の無い方向は大きな分散になる。floor は共分散に足す下限
	struct PlanarMeasurement {
		Vec3 pose{};
		Mat3 covariance{};
	};
	auto planar_measurement(const Belief& field_belief, const SE3& T_base_lidar, const Mat3& floor)
		-> PlanarMeasurement;

	/// フィールド物体の姿勢 (LiDAR から見たフィールド) から、機体の平面姿勢を求める
	auto base_pose_from_field(const SE3& M_field, const SE3& T_base_lidar) -> Vec3;
}
