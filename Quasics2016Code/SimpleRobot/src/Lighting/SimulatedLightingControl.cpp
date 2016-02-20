#include "SimulatedLightingControl.h"

const char * kStateNames[] = { "Red", "Blue", "Demo", "Error" };
const unsigned int kNumStateNames = sizeof(kStateNames)
		/ sizeof(kStateNames[0]);

const char * kModeNames[] = { "Solid", "Slow Blinking", "Blinking",
		"Fast Blinking", "Breathing" };
const unsigned int kNumModeNames = sizeof(kModeNames) / sizeof(kModeNames[0]);

std::ostream& operator<<(std::ostream& os, LightingControl::State s) {
	if (int(s) < kNumStateNames) {
		os << kStateNames[s];
	} else {
		os << "<UNKNOWN>";
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, LightingControl::Mode m) {
	if (int(m) < kNumModeNames) {
		os << kModeNames[m];
	} else {
		os << "<Unknown>";
	}
	return os;
}

void SimulatedLightingControl::SetState(State whichState) {
	std::cout << "Setting state to " << whichState << std::endl;
}

void SimulatedLightingControl::SetMode(Mode whichMode) {
	std::cout << "Setting mode to " << whichMode << std::endl;
}

void SimulatedLightingControl::SendHeartbeat() {
	std::cout << "Sending Heartbeat " << std::endl;
}

void SimulatedLightingControl::SendBatteryState(bool isLow) {
	std::cout << "Battery Status ";
	if (isLow) {
		std::cout << "Low";
	} else {
		std::cout << "Good";
	}
}
