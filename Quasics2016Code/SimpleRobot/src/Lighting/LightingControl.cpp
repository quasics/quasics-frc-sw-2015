/*
 * LightingControl.cpp
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#include "LightingControl.h"

LightingControl::LightingControl() {
	lastBatterySwitch = 0;
	lightingTimer = 0;
	isBatteryLow = false;
	previousBatteryState = false;
	previousState = kError;
	previousMode = kErrorMode;

	SmartDashboard::PutBoolean("In Competition?", false);
}

void LightingControl::LightingUpkeep() {
	//Timer Control
	lightingTimer++;	//Increment the lighting control timer

	//Establish local variables
	State state = kDemo;	//Local storage of state
	Mode mode = kBreathing;	//Local storage of mode

	//Status Check
	//Battery check
	if (DriverStation::GetInstance().GetBatteryVoltage() < 12//If the battery is low and it wasn't at the last check...
	&& previousBatteryState == false) {
		isBatteryLow = true;	//Set batteryState to low
		lastBatterySwitch = lightingTimer;	//Save the time stamp of the change
	} else if (DriverStation::GetInstance().GetBatteryVoltage() >= 12) {//Otherwise if the voltage is above 12 volts...
		isBatteryLow = false;	//Set battery state to good
		lastBatterySwitch = lightingTimer;	//Save the time stamp of the change
	}

	//State check
	if (!DriverStation::GetInstance().IsDSAttached()) {	//If the driver station isn't attached...
		state = kError;	//Set error state
	} else if (!SmartDashboard::GetBoolean("InCompetetion", false)) {//Otherwise if we aren't in competition...
		state = kDemo;	//Set demo state
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::GetInstance().kRed) {	//Otherwise if we are on the red alliance...
		state = kRedTeam; //Set red team state
	} else if (DriverStation::GetInstance().GetAlliance() //Otherwise if we are on the blue alliance...
	== DriverStation::GetInstance().kBlue) {
		state = kBlueTeam;	//Set blue team state
	} else {	//Otherwise...
		state = kError; //Set error state
	}

	//Mode Check
	if (DriverStation::GetInstance().IsDisabled()) {	//If we are disabled...
		mode = kBreathing;
	} else if (DriverStation::GetInstance().IsAutonomous()) {	//Otherwise if
		mode = kSolid;
	} else if (DriverStation::GetInstance().IsOperatorControl()) {
		mode = kSlowBlinking;
	} else if (DriverStation::GetInstance().IsTest()) {
		mode = kMediumBlink;
	} else {
		mode = kErrorMode;
	}

	//Data sending
	//State, mode, and battery sending
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
		if (previousBatteryState != isBatteryLow
				&& lightingTimer - lastBatterySwitch >= 500) {
			SendBatteryState(isBatteryLow);
		}
	}

	//Heart beat sending
	if (lightingTimer % 500 == 0) {
		SendHeartbeat();
	}


	//Save previous data
	previousBatteryState = isBatteryLow;
	previousState = state;
	previousMode = mode;
}
