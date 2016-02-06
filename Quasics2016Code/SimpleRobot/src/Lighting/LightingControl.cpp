/*
 * LightingControl.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#include "LightingControl.h"
#include <iostream>

LightingControl::LightingControl() {
	batteryTimer = 0;
	heartbeatTimer = 0;
	isBatteryLow = false;
	SmartDashboard::PutBoolean("In Competition?", false);
}

void LightingControl::LightingUpkeep() {
	heartbeatTimer++;
	if (heartbeatTimer >= 250) {
		SendHeartbeat();
		heartbeatTimer = 0;
	}

	if (DriverStation::GetInstance().GetBatteryVoltage() < 12) {
		batteryTimer++;
	} else {
		batteryTimer--;
	}

	if (batteryTimer >= 500) {
		isBatteryLow = true;
		if (batteryTimer >= 550) {
			batteryTimer = 599;
		}
	} else if (batteryTimer <= 400) {
		isBatteryLow = false;
	}

	if (!DriverStation::GetInstance().IsDSAttached() || isBatteryLow) {
		SetState(kError);
	} else if (SmartDashboard::GetBoolean("In Competition?", false)) {
		SetState(kDemo);
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::kRed) {
		SetState(kRedTeam);
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::kBlue) {
		SetState(kBlueTeam);
	}

	if (!DriverStation::GetInstance().IsDSAttached()) {
		SetMode(kQuickBlink);
	} else if (DriverStation::GetInstance().IsDisabled()) {
		SetMode(kBreathing);
	} else if (DriverStation::GetInstance().IsAutonomous()) {
		SetMode(kSolid);
	} else if (DriverStation::GetInstance().IsOperatorControl()) {
		SetMode(kSlowBlinking);
	} else if (DriverStation::GetInstance().IsTest()) {
		SetMode(kMediumBlink);
	} else {
		SetMode(kQuickBlink);
	}
}

const char * kStateNames[] = { "Red", "Blue", "Demo", "Error" };
const unsigned int kNumStateNames = sizeof(kStateNames)
		/ sizeof(kStateNames[0]);

const char * kModeNames[] = { "Solid", "Slow Blinking", "Blinking",
		"Fast Blinking", "Breathing" };
const unsigned int kNumModeNames = sizeof(kModeNames) / sizeof(kModeNames [0]);

std::ostream& operator<<(std::ostream& os, LightingControl::State s) {
	if (int(s) < kNumStateNames) {
		os << kStateNames[s];
	} else {
		os << "<UNKNOWN>";
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, LightingControl::Mode m){
	if (int (m) < kNumModeNames){
		os << kModeNames [m];
	} else {
		os << "<Unknown>";
	}
	return os;
}

void SimulatedLightingControl::SetState(State whichState) {
	std::cout << "Setting state to " << whichState << std::endl;
}

void SimulatedLightingControl::SetMode(Mode whichMode) {
	std::cout <<"Setting mode to " <<whichMode <<std::endl;
}

void SimulatedLightingControl::SendHeartbeat() {
	std::cout <<"Sending Heartbeat " <<std::endl;
}

void SimulatedLightingControl::SendBatteryState(bool isLow){
	std::cout <<"Battery Status ";
	if (isLow){
		std::cout <<"Low";
	} else {
		std::cout <<"Good";
	}
}
