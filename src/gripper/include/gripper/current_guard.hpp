#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace gripper {

// Sustained current in a commanded height direction learns a travel boundary.
// Only feedback samples advance the timer; missing samples never count as overload.
class CurrentGuard final {
public:
	using Clock = std::chrono::steady_clock;
	using TimePoint = Clock::time_point;
	CurrentGuard(double amps, double duration, double feedback_timeout)
		: amps_{amps}, duration_{duration}, feedback_timeout_{feedback_timeout} {
		if (!std::isfinite(amps) || amps <= 0.0 || !std::isfinite(duration) || duration <= 0.0
			|| !std::isfinite(feedback_timeout) || feedback_timeout <= 0.0) {
			throw std::invalid_argument("lift overcurrent threshold, duration and feedback timeout must be finite and positive");
		}
	}

	// Returns +1 for an upper stop, -1 for a lower stop, otherwise 0.
	int sample(double current, int direction, double height, TimePoint now) {
		if (!std::isfinite(current) || !std::isfinite(height) || (direction != -1 && direction != 1)
			|| std::abs(current) < amps_) {
			reset_timer();
			return 0;
		}
		if (!last_ || now < *last_ || std::chrono::duration<double>(now - *last_).count() > feedback_timeout_
			|| direction != direction_) {
			since_ = now;
		}
		last_ = now;
		direction_ = direction;
		if (std::chrono::duration<double>(now - *since_).count() < duration_) {
			return 0;
		}
		if (direction > 0) {
			upper_ = upper_ ? std::min(*upper_, height) : height;
		} else {
			lower_ = lower_ ? std::max(*lower_, height) : height;
		}
		reset_timer();
		return direction;
	}

	bool permits(double goal) const {
		return std::isfinite(goal) && (!upper_ || goal < *upper_) && (!lower_ || goal > *lower_);
	}
	void reset_timer() { since_.reset(); last_.reset(); direction_ = 0; }
	// Re-zeroing changes coordinates, not the physical limits already detected.
	void shift_origin(double old_height) {
		if (upper_) { *upper_ -= old_height; }
		if (lower_) { *lower_ -= old_height; }
		reset_timer();
	}
	const std::optional<double>& upper() const { return upper_; }
	const std::optional<double>& lower() const { return lower_; }

private:
	double amps_;
	double duration_;
	double feedback_timeout_;
	int direction_{0};
	std::optional<TimePoint> since_{};
	std::optional<TimePoint> last_{};
	std::optional<double> upper_{};
	std::optional<double> lower_{};
};

// ESC feedback uses signed raw current, including -32768.
inline double current_amps(std::int16_t raw, bool c620) {
	return static_cast<double>(raw) * (c620 ? 20.0 / 16384.0 : 10.0 / 10000.0);
}

} // namespace gripper
