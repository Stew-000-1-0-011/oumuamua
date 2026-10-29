#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace harite {

// ロータ角速度 [rad/s]。受信時刻は ROS 時刻ではなく単調時計を使う。
class VelocityCommand {
public:
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;

	VelocityCommand(double max_velocity, double timeout)
		: max_velocity_{max_velocity}, timeout_{timeout} {
		if (!std::isfinite(max_velocity) || max_velocity <= 0.0
			|| max_velocity > std::numeric_limits<float>::max()
			|| !std::isfinite(timeout) || timeout <= 0.0) {
			throw std::invalid_argument("max_velocity and command_timeout must be finite and positive (velocity must fit float32)");
		}
	}

	void enable(bool enabled) {
		if (enabled_ != enabled) {
			clear(); // 再有効化しても古い指令では動かさない
		}
		enabled_ = enabled;
	}
	bool enabled() const { return enabled_; }

	bool receive(double velocity, TimePoint now) {
		if (!enabled_ || !std::isfinite(velocity)) {
			clear();
			return false;
		}
		velocity_ = std::clamp(velocity, -max_velocity_, max_velocity_);
		received_ = now;
		return true;
	}

	double target(TimePoint now) const {
		if (!enabled_ || !received_ || now < *received_
			|| std::chrono::duration<double>(now - *received_) >= timeout_) {
			return 0.0;
		}
		return velocity_;
	}

private:
	void clear() { velocity_ = 0.0; received_.reset(); }
	double max_velocity_;
	std::chrono::duration<double> timeout_;
	bool enabled_{false};
	double velocity_{0.0};
	std::optional<TimePoint> received_;
};

} // namespace harite
