/// @file estimator_test.cpp
/// フィルタと信念の変換のテスト (ROS不要)。

#include <cmath>
#include <cstdio>
#include <numbers>
#include <source_location>

#include "state_estimator/belief_geometry.hpp"
#include "state_estimator/filter.hpp"

namespace {
	using namespace state_estimator;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol) -> bool { return std::abs(a - b) <= tol; }

	/// LiDAR を上下逆さに、少しずらして付けた取付
	auto mount() -> SE3 {
		return SE3{Sophus::SO3d::rotX(std::numbers::pi), Eigen::Vector3d{0.05, -0.02, 0.14}};
	}

	void test_integrates_velocity() {
		Filter f{};
		f.set_params(FilterParams{.tau_linear = 0.0, .tau_angular = 0.0});
		f.initialize(0.0, Vec3{0.0, 0.0, std::numbers::pi / 2}, Mat3::Identity() * 1e-4);
		f.add_input(0.0, Vec3{1.0, 0.0, 0.0});
		const auto s = f.predict(1.0);
		// +y を向いて前進すると、フィールドの +y に進む
		check(near(s->x(0), 0.0, 1e-6) && near(s->x(1), 1.0, 1e-6), "body velocity is integrated in the field frame");
		check(s->P(1, 1) > 1e-4, "uncertainty grows while predicting");
	}

	void test_lagged_input() {
		Filter f{};
		f.set_params(FilterParams{.tau_linear = 0.1, .tau_angular = 0.1});
		f.initialize(0.0, Vec3::Zero(), Mat3::Identity() * 1e-4);
		f.add_input(0.0, Vec3{1.0, 0.0, 0.0});
		const auto s = f.predict(0.1);
		check(near(s->x(3), 1.0 - std::exp(-1.0), 1e-6), "velocity follows the input with the time constant");
	}

	void test_delayed_measurement() {
		// 0.5 m/s で進んでいるが、実際には 0.1 m 先にいる。0.2 s 前の観測で直すと、今の推定も直る
		Filter f{};
		f.set_params(FilterParams{.tau_linear = 0.0, .tau_angular = 0.0, .pose_noise_linear = 1e-4});
		f.initialize(0.0, Vec3::Zero(), Mat3::Identity() * 0.01);
		f.add_input(0.0, Vec3{0.5, 0.0, 0.0});
		const double t_meas = 0.8;
		const auto r = f.update(t_meas, Vec3{0.5 * t_meas + 0.1, 0.0, 0.0}, Mat3::Identity() * 1e-6);
		check(r == UpdateResult::accepted, "measurement accepted");
		const auto s = f.predict(1.0);
		check(near(s->x(0), 0.5 + 0.1, 1e-3), "a delayed measurement corrects the current estimate");
		check(f.update(0.5, Vec3::Zero(), Mat3::Identity()) == UpdateResult::out_of_order, "older than the anchor is dropped");
	}

	void test_gate() {
		Filter f{};
		f.set_params(FilterParams{.gate_sigma = 3.0});
		f.initialize(0.0, Vec3::Zero(), Mat3::Identity() * 1e-4);
		check(f.update(0.1, Vec3{1.0, 0.0, 0.0}, Mat3::Identity() * 1e-4) == UpdateResult::gated, "a jump is gated");
	}

	void test_prior_roundtrip() {
		// 機体の姿勢 -> フィールド物体の事前 -> 機体の姿勢 で元に戻る
		const Vec3 base{0.3, -0.2, 0.7};
		const SE3 T_field_field{};  // フィールド物体はフィールド座標系そのもの
		const Mat3 cov = Vec3{1e-4, 2e-4, 3e-4}.asDiagonal();
		const auto prior = make_prior(base, cov, mount(), T_field_field, PriorNoise{});
		const Vec3 back = base_pose_from_field(prior.mean, mount());
		check((back - base).norm() < 1e-9, "base pose survives the round trip through the prior");

		// 事前の情報を平面の観測に戻すと、元の共分散になる (面外の不確かさは平面に漏れない)
		const auto meas = planar_measurement(prior, mount(), Mat3::Zero());
		check((meas.covariance - cov).norm() < 1e-8, "prior information maps back to the planar covariance");
	}

	void test_out_of_plane_unobserved() {
		// 2D LiDAR の観測: 面外 (センサ座標系の z と roll, pitch) は情報 0。
		// 上下逆さで床から離れた LiDAR でも、見えない傾きが水平位置の不確かさに漏れないこと
		const Vec3 truth{0.3, -0.2, 0.7};
		const auto full = make_prior(truth, Vec3{1e-6, 1e-6, 1e-6}.asDiagonal(), mount(), SE3{}, PriorNoise{});
		Mat6 L = full.information;
		for (const int k : {2, 3, 4}) {
			L.row(k).setZero();
			L.col(k).setZero();
		}
		const auto meas = planar_measurement(Belief{full.mean, L}, mount(), Mat3::Zero());
		check((meas.pose - truth).norm() < 1e-6, "planar pose is recovered without out-of-plane information");
		check(meas.covariance.diagonal().maxCoeff() < 1e-5, "unobserved tilt does not leak into the planar covariance");
	}

	void test_remove_prior() {
		// スキャンだけの信念 (x 方向は見えない) と事前を合成した事後から、事前を差し引くとスキャンだけに戻る
		const SE3 T_field_field{};
		const Vec3 truth{0.3, -0.2, 0.7};
		const auto scan_full = make_prior(truth, Vec3{1e-6, 1e-6, 1e-6}.asDiagonal(), mount(), T_field_field, PriorNoise{});
		// センサ座標系の並進 x を見えなくする
		Mat6 L_scan = scan_full.information;
		L_scan.row(0).setZero();
		L_scan.col(0).setZero();
		const Belief scan{scan_full.mean, L_scan};

		const auto prior = make_prior(Vec3{0.32, -0.21, 0.69}, Vec3{4e-4, 4e-4, 1e-3}.asDiagonal(), mount(), T_field_field, PriorNoise{});

		// 事後 (スキャンの平均のまわりの局所座標で合成)
		const Vec6 xi_pri = (prior.mean * scan.mean.inverse()).log();
		const Mat6 L_post = prior.information + scan.information;
		const Vec6 xi_post = L_post.ldlt().solve(prior.information * xi_pri);
		const Belief posterior{SE3::exp(xi_post) * scan.mean, L_post};

		const auto recovered = remove_prior(posterior, prior);
		check((recovered.information - L_scan).norm() < 1e-6 * L_scan.norm(), "scan information is recovered");
		// 見えている方向の平均が戻る (見えない方向は情報 0 なので比べない)
		const Vec6 err = (recovered.mean * scan.mean.inverse()).log();
		check((L_scan * err).norm() < 1e-3 * L_scan.norm(), "scan mean is recovered in the observed directions");
	}
}

auto main() -> int {
	test_integrates_velocity();
	test_lagged_input();
	test_delayed_measurement();
	test_gate();
	test_prior_roundtrip();
	test_remove_prior();
	test_out_of_plane_unobserved();
	if (failures == 0) {
		std::printf("all estimator tests passed\n");
		return 0;
	}
	std::printf("%d failure(s)\n", failures);
	return 1;
}
