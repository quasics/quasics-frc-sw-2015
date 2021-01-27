// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "../../../../Common2021/TeleopTankDrive.h"
#include "Constants.h"

inline double DeadBand(double stickValue) {
  if (stickValue > OIConstants::DeadBand_LowValue &&
      stickValue < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return (stickValue);
}

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
  TeleopTankDrive tankDrive(
      &m_driveBase,
      [this] {
        return DeadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis));
      },
      [this] {
        return DeadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::LeftYAxis));
      });
  m_driveBase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
