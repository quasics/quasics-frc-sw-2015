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
  auto& driverStation = frc::DriverStation::GetInstance();

  if (!driverStation.IsDSAttached()) {
    // Send command to signal that the Driver Station isn't connected (yet).
    transmitStatus("no-ds");
    return;
  }

  // TODO: Figure out if we want to do anything with the rest of this state.
  const bool enabled = driverStation.IsEnabled();
  const auto ds_alliance = driverStation.GetAlliance();
  const bool fmsAttached = driverStation.IsFMSAttached();

  const bool brownout = frc::RobotController::IsBrownedOut();
  if (brownout) {
    transmitStatus("brownout");
    return;
  }

  const bool autonomous = driverStation.IsAutonomous();
  const bool teleop = driverStation.IsOperatorControl();
  std::string modeCmd;
  if (autonomous) {
    modeCmd += "mode:auto";
  } else if (teleop) {
    modeCmd += "mode:teleop";
  } else {
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

  // Send commands to set lighting mode to indicate current robot status.
  //
  // Assumption is that the different piece of status will all be strung
  // together into a single command, with the pieces separated by semicolons.
  transmitStatus(modeCmd + ';' + colorCmd);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLighting::IsFinished() { return false; }

void AutoLighting::transmitStatus(std::string statusCommand, bool alwaysSend) {
  if (alwaysSend || statusCommand != lastCommand) {
    std::cout << "Sending command: '" << statusCommand << "'" << std::endl;
    // If the command (state) has changed, or if we're forcing a re-send, pass
    // the current status on to the Arduino.
    if (!Robot::m_lighting.sendCommandToArduino(statusCommand)) {
      std::cerr << "--- Error sending command!\n";
    }
  }

  // Remember for the next time, so that we don't send the same command 50 times
  // per second.
  lastCommand = statusCommand;
}