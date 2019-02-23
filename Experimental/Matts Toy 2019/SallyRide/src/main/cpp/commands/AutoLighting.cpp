/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoLighting.h"
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <iostream>
#include "Robot.h"

AutoLighting::AutoLighting() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_lighting);
}

// Called repeatedly when this Command is scheduled to run
void AutoLighting::Execute() {
  // If we're browned out, then signal that ('cause that's what's most
  // important in this case).
  const bool brownout = frc::RobotController::IsBrownedOut();
  if (brownout) {
    transmitStatusCommands("brownout");
    return;
  }

  // Try to get the driver's station object, so that we can gather information
  // to consider sending to the Arduino.
  auto& driverStation = frc::DriverStation::GetInstance();
  if (!driverStation.IsDSAttached()) {
    // Send command to signal that the Driver Station isn't connected (yet).
    transmitStatusCommands("no-ds");
    return;
  }

  // Examples of the kinds of information that we might want to pass along.
  //
  // TODO: Figure out if we want to do anything with the rest of this state.
  // For example, we might use the fact that we're in an elimination match
  // to tell the Arduino to "be more excited" and change the lighting faster.
  //
  // Or we could use the "elapsed match time" for the same sort of thing, so
  // that as the match progresses, we "get more excited".
  const bool enabled = driverStation.IsEnabled();
  const bool disabled = driverStation.IsDisabled();
  const auto ds_alliance = driverStation.GetAlliance();
  const bool fmsAttached = driverStation.IsFMSAttached();

  // Match type will be one of: kNone, kPractice, kQualification, kElimination
  const auto matchType = driverStation.GetMatchType();
  const int matchNumber = driverStation.GetMatchNumber();
  const int replayNumber = driverStation.GetReplayNumber();
  const int driverLocation = driverStation.GetLocation();  // 1, 2, 3

  // Approx. time elapsed in match
  const double elapsedMatchTime = driverStation.GetMatchTime();

  // At most one of the following should be true at any point.
  const bool inAutoMode = driverStation.IsAutonomous();
  const bool inTeleopMode = driverStation.IsOperatorControl();
  const bool testMode = driverStation.IsTest();

  ////////////////////////////////////////////////////////////////////////////
  // Using the data we've gathered above, figure out what to tell the Arduino.
  std::string modeCmd;
  if (inAutoMode) {
    modeCmd += "mode:auto";
  } else if (inTeleopMode) {
    modeCmd += "mode:teleop";
  } else {
    // We won't bother handling "test" mode for now.
    modeCmd += "mode:unknown";
  }

  std::string color = "default";
  switch (ds_alliance) {
    case frc::DriverStation::kRed:
      color = "red";
      break;
    case frc::DriverStation::kBlue:
      color = "blue";
      break;
    case frc::DriverStation::kInvalid:
      color = "demo";
      break;
  }
  const std::string colorCmd = "color:" + color;

  ////////////////////////////////////////////////////////////////////////////
  // Send commands to set lighting mode to indicate current robot status.

  // The assumption that we're making in this example is that the different
  // piece of status will all be strung together into a single transmission,
  // with the pieces separated by semicolons.
  //
  // An alternative would be to send them out separately, but then we'd need to
  // keep track of "did we send this last time?" for each of them independently.
  // (Which is certainly doable, but means more "lasts" to keep track of.)
  std::string statusCommandToSend = modeCmd + ';' + colorCmd;

  transmitStatusCommands(statusCommandToSend);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLighting::IsFinished() { return false; }

void AutoLighting::transmitStatusCommands(std::string statusCommand,
                                          bool alwaysSend) {
  if (statusCommand != lastStatusCommand && !alwaysSend) {
    // This is the same thing that we sent out last time, and we aren't forcing
    // retransmission, so we won't bother sending it again.
    return;
  }

  // Try to send it out.
  std::cout << "Sending command: '" << statusCommand << "'" << std::endl;
  if (!Robot::m_lighting.sendCommandToArduino(statusCommand)) {
    std::cerr << "--- Error sending command!\n";
  }

  // Remember what we sent, to allow checking the next time around.
  lastStatusCommand = statusCommand;
}