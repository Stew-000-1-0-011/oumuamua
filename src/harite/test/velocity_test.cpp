#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "harite/velocity_command.hpp"

namespace {
void require(bool condition, const char* message) {
	if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
}

int main() {
	using harite::VelocityCommand;
	using namespace std::chrono_literals;
	const auto t = VelocityCommand::TimePoint{} + 1s;
	VelocityCommand command{100.0, 0.2};
	require(command.target(t) == 0.0, "startup must be stopped");
	require(!command.receive(20.0, t), "disabled command must be ignored");
	command.enable(true);
	require(command.target(t) == 0.0, "enable must not replay a disabled command");
	require(command.receive(20.0, t), "valid command rejected");
	require(command.target(t + 199ms) == 20.0, "fresh command lost");
	require(command.target(t + 200ms) == 0.0, "timeout boundary must stop");
	require(command.target(t + 10s) == 0.0, "stale command resumed");
	command.receive(150.0, t);
	require(command.target(t) == 100.0, "positive speed limit");
	command.receive(-150.0, t);
	require(command.target(t) == -100.0, "negative speed limit");
	command.enable(true);
	require(command.target(t) == -100.0, "repeated enable should be idempotent");
	command.enable(false);
	require(command.target(t) == 0.0, "disable must stop immediately");
	command.receive(50.0, t);
	command.enable(true);
	require(command.target(t) == 0.0, "re-enable must require a fresh command");
	for (double value : {std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
		command.receive(50.0, t);
		require(!command.receive(value, t + 10ms), "invalid input accepted");
		require(command.target(t + 10ms) == 0.0, "invalid input must clear motion");
	}
	command.receive(50.0, t);
	require(command.target(t - 1ms) == 0.0, "negative elapsed time must stop");
	command.receive(0.0, t);
	require(command.target(t) == 0.0, "zero command must stop");
	for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::quiet_NaN()}) {
		bool rejected = false;
		try { VelocityCommand bad{value, 0.2}; } catch (const std::invalid_argument&) { rejected = true; }
		require(rejected, "invalid velocity limit accepted");
		rejected = false;
		try { VelocityCommand bad{100.0, value}; } catch (const std::invalid_argument&) { rejected = true; }
		require(rejected, "invalid timeout accepted");
	}
	bool rejected = false;
	try { VelocityCommand bad{std::numeric_limits<double>::max(), 0.2}; }
	catch (const std::invalid_argument&) { rejected = true; }
	require(rejected, "float32 overflow limit accepted");
	std::cout << "harite velocity tests passed\n";
}
