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
    return;
  }

  if (!driverStation.IsFMSAttached()) {
    // Send command to indicate that we don't have an FMS to tell us what's
    // going on.
    return;
  }

  const bool enabled = driverStation.IsEnabled();
  const bool autonomous = driverStation.IsAutonomous();
  const bool teleop = driverStation.IsOperatorControl();
  const bool brownout = frc::RobotController::IsBrownedOut();
  const auto ds_alliance = driverStation.GetAlliance();

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

  // Send commands to set lighting mode to indicate robot status
  // e.g.,
  if (!Robot::m_lighting.sendCommandToArduino("color-" + color)) {
    std::cerr << "Failed to send color command for '" << color << "'\n";
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLighting::IsFinished() { return false; }
