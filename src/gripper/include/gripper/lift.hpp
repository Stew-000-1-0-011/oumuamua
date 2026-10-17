#pragma once

/// 昇降の、ROS に依存しない部分。
///
/// - ロボマス (C610 / C620) のフィードバック (CAN 0x200 + モータ番号) の読み取り
/// - ロータの角度 (0..8191) を数え上げて、起動時からの回転量にする
/// - 位置指令を、ゴールへ向けて最高速度で動かす

#include <array>
#include <cstdint>
#include <optional>

namespace gripper {
	/// ロボマスの ESC (C610 / C620) が 1 kHz で出す状態。モータ番号 k なら CAN ID は 0x200 + k
	struct RobomasFeedback final {
		std::uint16_t angle;  ///< ロータの角度 [0, 8191]
		std::int16_t rpm;	  ///< ロータの回転数 [rpm]
		std::int16_t current; ///< トルク電流 (C620 は ±16384 が ±20 A、C610 は ±10000 が ±10 A)
		std::uint8_t temperature; ///< [℃] (C610 は 0)
	};

	/// CAN ID が motor (1..8) のフィードバックなら読む
	auto parse_feedback(std::uint32_t id, std::uint8_t dlc, const std::array<std::uint8_t, 8>& data, int motor)
		-> std::optional<RobomasFeedback>;

	/// ロータの角度を数え上げて、最初に受け取った角度からの回転量 [回転] にする。
	/// 1 周期 (1 ms) の間に半周以上は回らない前提
	class RotorCounter final {
	public:
		void update(std::uint16_t angle);
		auto valid() const -> bool { return this->last_.has_value(); }
		/// 最初の角度からのロータの回転量 [回転]
		auto revolutions() const -> double;
		void reset() { this->last_.reset(); }

	private:
		std::optional<std::uint16_t> last_{};
		std::int64_t counts_{0};
	};

	/// 位置指令を、ゴールへ向けて max_speed で動かす
	class Ramp final {
	public:
		explicit Ramp(double position = 0.0) : position_{position}, goal_{position} {}
		void set_goal(double goal) { this->goal_ = goal; }
		/// 今の位置指令でゴールを置き換える (止める)
		void hold() { this->goal_ = this->position_; }
		/// dt [s] だけ進めて、新しい位置指令を返す
		auto step(double dt, double max_speed) -> double;
		auto position() const -> double { return this->position_; }
		auto goal() const -> double { return this->goal_; }
		auto arrived() const -> bool { return this->position_ == this->goal_; }

	private:
		double position_;
		double goal_;
	};
} // namespace gripper
