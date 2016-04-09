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

LightingControl::State LightingControl::ComputeCurrentState() const {
    // Establish local variables for current state
    State result = kDemo;

    // State check
    if (!DriverStation::GetInstance().IsDSAttached()) {
        // Driver station isn't attached
        result = kError;
    }
    else if (!DriverStation::GetInstance().IsFMSAttached()) {
        // Not in competition (demo mode)
        result = kDemo;
    }
    else if (DriverStation::GetInstance().GetAlliance()
             == DriverStation::GetInstance().kRed) {
        // Red alliance
        result = kRedTeam;
    }
    else if (DriverStation::GetInstance().GetAlliance()
             == DriverStation::GetInstance().kBlue) {
        // Blue alliance
        result = kBlueTeam;
    }
    else if (DriverStation::GetInstance().GetMatchTime() <= 30
             && DriverStation::GetInstance().IsOperatorControl()) {
        result = kDemo;
    }
    else {
        // Shouldn't be possible: consider it an error
        result = kError;
    }

    return result;
}

LightingControl::Mode LightingControl::ComputeCurrentMode() const {
    Mode result = kBreathing;

    if (DriverStation::GetInstance().IsDisabled()) {
        // Disabled
        result = kBreathing;
    }
    else if (DriverStation::GetInstance().IsAutonomous()) {
        // Autonomous
        result = kSolid;
    }
    else if (DriverStation::GetInstance().IsOperatorControl()) {
        // Teleop
        result = kSlowBlinking;
    }
    else if (DriverStation::GetInstance().IsTest()) {
        // Test
        result = kMediumBlink;
    }
    else {
        // ???
        result = kErrorMode;
    }

    return result;
}

void LightingControl::LightingUpkeep() {
    //
    // Timer update
    lightingTimer++;    //Increment the lighting control timer
    // CODE_REVIEW(mjh): Have you thought about how to handle rollover/overflow?
    // If this isn't an issue, then you should probably document it as such
    // (and why).  You might also think about moving timer management into a
    // simple utility class, rather than manipulating it here.

    //
    // Status update/calculation

    // Get the state/mode values that will be used to drive the lights.
    const State state = ComputeCurrentState();
    const Mode mode = ComputeCurrentMode();

    // Battery check
    bool isBatteryLow = previousBatteryIsLow;
    if (DriverStation::GetInstance().GetBatteryVoltage() < 12
            && !previousBatteryIsLow) {
        // Battery has just gone into "low" condition.
        isBatteryLow = true;
        lastBatterySwitch = lightingTimer;
    }
    else if (DriverStation::GetInstance().GetBatteryVoltage() >= 12) {
        // The voltage is above 12 volts...
        isBatteryLow = false;   //Set battery state to good
        lastBatterySwitch = lightingTimer;  //Save the time stamp of the change
    }

    //
    // Send current settings to the control subsystem (e.g., the Arduino).
    if (lightingTimer % 1500 == 0) {
        // Every so often, we make sure that the state/mode are update to date,
        // just to be safe.
        SetState(state);
        SetMode(mode);

        if (lightingTimer - lastBatterySwitch >= 500) {
            SendBatteryState(isBatteryLow);
        }
    }
    else {
        // Only send the changes
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

    // Heart beat sending
    if (lightingTimer % 100 == 0) {
        SendHeartbeat();
    }

    //
    //Save current status for use during next iteration
    previousBatteryIsLow = isBatteryLow;
    previousState = state;
    previousMode = mode;
}
