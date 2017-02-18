#include "Lighting.h"
#include "../RobotMap.h"

Lighting::Lighting() : Subsystem("Lighting"), alliance_(eDemo), mode_(eIdle) {
	serialPort_.reset(new SerialPort(115200, SerialPort::kMXP));
}

void Lighting::InitDefaultCommand() {
}

const char* const kLightMapping[3][4] = {
	{"breathe-red\n", "updown-red\n", "chase-red\n", "breath-yellow\n"},
	{"breathe-blue\n", "updown-blue\n", "chase-blue\n", "breath-yellow\n"},
	{"breathe-green\n", "updown-green\n", "chase-green\n", "breath-yellow\n"},
};

void Lighting::setAlliance(Alliance alliance) {
	this->alliance_ = alliance;
	sendLightingCommand();
}

void Lighting::setMode(Mode mode) {
	this->mode_ = mode;
	sendLightingCommand();
}

void Lighting::sendLightingCommand() {
	if (serialPort_) {
		serialPort_->Write(kLightMapping[alliance_][mode_]);
	}
}
