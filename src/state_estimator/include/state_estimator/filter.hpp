#pragma once
/// @file filter.hpp
/// 平面の姿勢と機体速度を持つ拡張カルマンフィルタ。ROS非依存。
///
/// 状態は [x, y, yaw, vx, vy, omega]。姿勢はフィールド座標系、速度は機体座標系。
///
/// 予測の入力は「下位層が推定した機体速度」u (機体座標系)。
///   tau > 0: dv/dt = (u - v) / tau + 白色雑音   (u が指令値で、実際の速度が遅れて追従するとき)
///   tau = 0: v = u                              (u が実際の速度の推定値 (車輪オドメトリなど) のとき)
/// 姿勢は v を向きで回して積分し、さらに姿勢そのものに白色雑音 (滑りなど) を足す。
///
/// 姿勢の観測は遅れて届く (スキャン時刻 + 処理時間)。観測は「最後に観測を入れた時刻の状態 (アンカー)」から
/// その時刻まで入力の履歴で予測して入れ、現在の推定はアンカーから入力の履歴で予測し直す。
/// アンカーより古い観測は捨てる。

#include <deque>
#include <optional>

#include <Eigen/Core>

namespace state_estimator {
	using Vec3 = Eigen::Vector3d;
	using Vec6 = Eigen::Matrix<double, 6, 1>;
	using Mat3 = Eigen::Matrix3d;
	using Mat6 = Eigen::Matrix<double, 6, 6>;

	struct FilterParams {
		/// 入力への追従の時定数 [s]。<= 0 なら v = u
		double tau_linear{0.0};
		double tau_angular{0.0};
		/// tau > 0 のときの速度の雑音 (スペクトル密度) [(m/s)^2/s], [(rad/s)^2/s]
		double velocity_noise_linear{1.0};
		double velocity_noise_angular{4.0};
		/// tau = 0 のときの入力 (速度) の標準偏差 [m/s], [rad/s]。推定の速度の不確かさとして出すだけ
		double input_sigma_linear{0.05};
		double input_sigma_angular{0.1};
		/// 姿勢の雑音 (スペクトル密度) [m^2/s], [rad^2/s]。入力の誤差や滑りのぶん
		double pose_noise_linear{0.01};
		double pose_noise_angular{0.01};
		/// 積分の刻み [s]
		double max_step{0.005};
		/// ゲート (マハラノビス距離)。<= 0 で無効
		double gate_sigma{0.0};
		/// アンカーから現在までをこれより長く予測しない [s]。超えたらアンカーを進める
		double max_horizon{0.5};
	};

	struct State {
		double stamp{};
		Vec6 x{Vec6::Zero()};
		Mat6 P{Mat6::Identity()};
	};

	enum class UpdateResult {
		accepted,
		out_of_order, ///< アンカーより古いので捨てた
		gated,        ///< ゲートで棄却した
		not_initialized,
	};

	class Filter {
	public:
		void set_params(const FilterParams& params) { this->params_ = params; }
		auto params() const -> const FilterParams& { return this->params_; }

		/// 時刻 t に姿勢 (と共分散) で初期化する。速度は 0
		void initialize(double t, const Vec3& pose, const Mat3& pose_cov);
		void reset() {
			this->anchor_.reset();
			this->inputs_.clear();
		}
		auto initialized() const -> bool { return this->anchor_.has_value(); }

		/// 時刻 t 以降の入力 (機体座標系の速度)。次の入力まで一定とみなす
		void add_input(double t, const Vec3& body_velocity);

		/// 時刻 t の姿勢の観測 z (フィールド座標系) と、その共分散 R
		auto update(double t, const Vec3& z, const Mat3& R) -> UpdateResult;

		/// 時刻 t の推定 (アンカーから入力の履歴で予測)。t がアンカーより古ければアンカーそのもの
		auto predict(double t) const -> std::optional<State>;

		/// 不確かさを足す (見失いかけたとき)。姿勢の共分散に加える
		void inflate(const Mat3& pose_cov);

		/// 長く観測が無いとき、予測が長くなりすぎないようにアンカーを進める
		void advance_anchor(double now);

		auto anchor_stamp() const -> std::optional<double> {
			return this->anchor_ ? std::optional<double>{this->anchor_->stamp} : std::nullopt;
		}

	private:
		struct Input {
			double t;
			Vec3 u;
		};

		/// state を dt だけ、入力 u で進める
		void step(State& state, const Vec3& u, double dt) const;
		/// 時刻 t の入力 (ZOH)
		auto input_at(double t) const -> Vec3;
		void prune_inputs();

		FilterParams params_{};
		std::optional<State> anchor_{};
		std::deque<Input> inputs_{};
	};

	/// 角度を [-pi, pi) に折り返す
	auto wrap_angle(double a) -> double;
}
