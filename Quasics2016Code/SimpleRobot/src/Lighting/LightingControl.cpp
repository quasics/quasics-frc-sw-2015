/*
 * LightingControl.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#include "LightingControl.h"

const char kInCompetitionDashboardFlag[] = "In Competition?";

LightingControl::LightingControl() {
	lastBatterySwitch = 0;
	lightingTimer = 0;
	previousBatteryIsLow = false;
	previousState = kError;
	previousMode = kErrorMode;

	SmartDashboard::PutBoolean(kInCompetitionDashboardFlag, false);
}

void LightingControl::LightingUpkeep() {
	// Timer Control
	lightingTimer++;	//Increment the lighting control timer

	// Establish local variables for current state
	State state = kDemo;
	Mode mode = kBreathing;

	//Status Check
	//Battery check
	bool isBatteryLow = previousBatteryIsLow;
	if (DriverStation::GetInstance().GetBatteryVoltage() < 12
			&& !previousBatteryIsLow) {
		// Battery has just gone into "low" condition.
		isBatteryLow = true;
		lastBatterySwitch = lightingTimer;
	} else if (DriverStation::GetInstance().GetBatteryVoltage() >= 12) {
		// The voltage is above 12 volts...
		isBatteryLow = false;	//Set battery state to good
		lastBatterySwitch = lightingTimer;	//Save the time stamp of the change
	}

	// State check
	if (!DriverStation::GetInstance().IsDSAttached()) {
		// Driver station isn't attached
		state = kError;
	} else if (!DriverStation::GetInstance().IsFMSAttached()) {
		// Not in competition (demo mode)
		state = kDemo;
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::GetInstance().kRed) {
		// Red alliance
		state = kRedTeam;
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::GetInstance().kBlue) {
		// Blue alliance
		state = kBlueTeam;
	} else if (DriverStation::GetInstance().GetMatchTime() <= 30
			&& DriverStation::GetInstance().IsOperatorControl()) {
		state = kDemo;
	} else {
		// Shouldn't be possible: consider it an error
		state = kError;
	}

	// Mode Check
	if (DriverStation::GetInstance().IsDisabled()) {
		// Disabled
		mode = kBreathing;
	} else if (DriverStation::GetInstance().IsAutonomous()) {
		// Autonomous
		mode = kSolid;
	} else if (DriverStation::GetInstance().IsOperatorControl()) {
		// Teleop
		mode = kSlowBlinking;
	} else if (DriverStation::GetInstance().IsTest()) {
		// Test
		mode = kMediumBlink;
	} else {
		// ???
		mode = kErrorMode;
	}

	// Data sending
	// State, mode, and battery sending
	if (lightingTimer % 1500 == 0) {
		SetState(state);
		SetMode(mode);
		if (lightingTimer - lastBatterySwitch >= 500) {
			SendBatteryState(isBatteryLow);
		}
	} else {
		if (state != previousState) {
			SetState(state);
		}
		if (mode != previousMode) {
			SetMode(mode);
		}
		if (previousBatteryIsLow != isBatteryLow
				&& lightingTimer - lastBatterySwitch >= 500) {
			SendBatteryState(isBatteryLow);
		}
	}

	//Heart beat sending
	if (lightingTimer % 100 == 0) {
		SendHeartbeat();
	}

	//Save previous data
	previousBatteryIsLow = isBatteryLow;
	previousState = state;
	previousMode = mode;
}
