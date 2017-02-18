#include "Lighting.h"
#include "../RobotMap.h"
#include "../Commands/AutomaticLightingCommand.h"

Lighting::Lighting() : Subsystem("Lighting"), alliance_(eDemo), mode_(eIdle) {
	serialPort_.reset(new SerialPort(115200, SerialPort::kMXP));
	updateState();
}

void Lighting::InitDefaultCommand() {
	SetDefaultCommand(new AutomaticLightingCommand);
}

void Lighting::updateState(bool force) {
	auto& driverStation = DriverStation::GetInstance();

	auto ds_alliance = driverStation.GetAlliance();
	switch (ds_alliance) {
		case DriverStation::kRed:
			alliance_ = eRed;
			break;
		case DriverStation::kBlue:
			alliance_ = eBlue;
			break;
		case DriverStation::kInvalid:
			alliance_ = eDemo;
			break;
	}

	if (driverStation.IsAutonomous()) {
		mode_ = eAuto;
	} else if (driverStation.IsOperatorControl()) {
		mode_ = eTeleOp;
	} else if (driverStation.IsTest()) {
		mode_ = eTest;
	} else if (driverStation.IsBrownedOut() || !driverStation.IsSysActive()) {
		mode_ = eError;
	} else {
		mode_ = eIdle;
	}

	sendLightingCommand(force);
}

const char* const kLightMapping[3][5] = {
	/* Idle */          /* Teleop */      /* Auto */       /* Error */        /* Test */
	{"breathe-red\n",   "updown-red\n",   "chase-red\n",   "breath-yellow\n", "flash-red\n"},
	{"breathe-blue\n",  "updown-blue\n",  "chase-blue\n",  "breath-yellow\n", "flash-blue\n"},
	{"breathe-green\n", "updown-green\n", "chase-green\n", "breath-yellow\n", "flash-green\n"},
};

void Lighting::setAlliance(Alliance alliance) {
	this->alliance_ = alliance;
	sendLightingCommand(false);
}

void Lighting::setMode(Mode mode) {
	this->mode_ = mode;
	sendLightingCommand(false);
}

void Lighting::sendLightingCommand(bool force) {
	const char * cmd = kLightMapping[alliance_][mode_];
	const bool canSendCommand = cmd && serialPort_;

	// Note that if we have a timeout on the Arduino (handling brownout, reset, etc.),
	// then we may want to force resending the command every so often, even if it has
	// not changed.
	const bool shouldSendCommand = (cmd != lastCommand_) || force;

	if (canSendCommand && shouldSendCommand) {
		serialPort_->Write(cmd);
		lastCommand_ = cmd;
	}
}
