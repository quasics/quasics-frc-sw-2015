/*
 * LightingControl.cpp
 *
 *  Created on: Nov 7, 2017
 *      Author: healym
 */

#include "LightingControl.h"
#include "../Commands/AutomaticLighting.h"
#include <iostream>

LightingControl::LightingControl()
: Subsystem("Lighting control")
{
	serialPort_.reset(new SerialPort(115200, SerialPort::kMXP));
	updateState();
}

void LightingControl::InitDefaultCommand() {
	SetDefaultCommand(new AutomaticLighting);
}

void LightingControl::updateState(bool force) {
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
	} else if (RobotController::IsBrownedOut() || !RobotController::IsSysActive()) {
		mode_ = eError;
	} else {
		mode_ = eIdle;
	}

	sendLightingCommand(force);
}

void LightingControl::setAlliance(Alliance alliance) {
	this->alliance_ = alliance;
	sendLightingCommand(false);
}

void LightingControl::setMode(Mode mode) {
	this->mode_ = mode;
	sendLightingCommand(false);
}

const char* const kLightMapping[3 /* alliance */][5 /* mode */] = {
				/* Idle */          /* Teleop */      /* Auto */       /* Error */        /* Test */
/* Red */		{"breathe-red\n",   "updown-red\n",   "chase-red\n",   "breath-yellow\n", "flash-red\n"},
/* Blue */		{"breathe-blue\n",  "updown-blue\n",  "chase-blue\n",  "breath-yellow\n", "flash-blue\n"},
/* Invalid */	{"breathe-green\n", "updown-green\n", "chase-green\n", "breath-yellow\n", "flash-green\n"},
};

// TODO: Determine what the commands to the Arduino will look like.
// The following code is based on some sample ideas, and the modes defined above.
void LightingControl::sendLightingCommand(bool force) {
	const char * cmd = kLightMapping[alliance_][mode_];
	const bool canSendCommand = cmd && serialPort_;

	// Note that if we have a timeout on the Arduino (handling brownout, reset, etc.),
	// then we may want to force re-sending the command every so often, even if it has
	// not changed.
	const bool shouldSendCommand = (cmd != lastCommand_) || force;

	if (canSendCommand && shouldSendCommand) {
		std::cout << "Sending lighting command: " << cmd;
		serialPort_->Write(cmd);
		lastCommand_ = cmd;
	}
}
