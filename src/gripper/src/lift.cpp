#include "gripper/lift.hpp"

#include <algorithm>
#include <cmath>

namespace gripper {
	auto parse_feedback(
		const std::uint32_t id, const std::uint8_t dlc, const std::array<std::uint8_t, 8>& data, const int motor
	) -> std::optional<RobomasFeedback> {
		if (motor < 1 || motor > 8 || id != 0x200u + static_cast<std::uint32_t>(motor) || dlc < 7) {
			return std::nullopt;
		}
		const auto u16 = [&](const int i) {
			return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[i]) << 8) | data[i + 1]);
		};
		return RobomasFeedback{
			.angle = static_cast<std::uint16_t>(u16(0) & 0x1fff),
			.rpm = static_cast<std::int16_t>(u16(2)),
			.current = static_cast<std::int16_t>(u16(4)),
			.temperature = data[6],
		};
	}

	void RotorCounter::update(const std::uint16_t angle) {
		if (this->last_) {
			// 一番近い向きに回ったとみなす (-4096..4095)
			int delta = static_cast<int>(angle) - static_cast<int>(*this->last_);
			if (delta >= 4096) {
				delta -= 8192;
			} else if (delta < -4096) {
				delta += 8192;
			}
			this->counts_ += delta;
		}
		this->last_ = angle;
	}

	auto RotorCounter::revolutions() const -> double { return static_cast<double>(this->counts_) / 8192.0; }

	auto Ramp::step(const double dt, const double max_speed) -> double {
		const double max_step = std::max(0.0, max_speed * dt);
		const double error = this->goal_ - this->position_;
		if (std::abs(error) <= max_step) {
			this->position_ = this->goal_;
		} else {
			this->position_ += std::copysign(max_step, error);
		}
		return this->position_;
	}
} // namespace gripper
