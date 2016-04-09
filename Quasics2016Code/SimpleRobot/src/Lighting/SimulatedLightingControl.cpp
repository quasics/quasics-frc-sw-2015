#include "SimulatedLightingControl.h"

// CODE_REVIEW(mjh): These names don't seem to match what you're using in the
// actual "ModeNames" type, and in the SerialLightingControl class.  (There's
// a nice trick that can be used to keep them all in sync that I can show you.)
const char * kStateNames[] = { "Red", "Blue", "Demo", "Error" };
const char * kModeNames[] = { "Solid", "Slow Blinking", "Blinking",
		"Fast Blinking", "Breathing" };

const unsigned int kNumStateNames =
    sizeof(kStateNames) / sizeof(kStateNames[0]);
const unsigned int kNumModeNames =
    sizeof(kModeNames) / sizeof(kModeNames[0]);

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
