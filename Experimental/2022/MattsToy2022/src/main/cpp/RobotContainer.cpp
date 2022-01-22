// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "commands/TankDrive.h"
#include "utils/DeadBandEnforcer.h"

RobotContainer::RobotContainer() {
  DeadBandEnforcer driverDeadBand{Deadbands::DRIVING};
  TankDrive tankDrive{&m_driveBase,
                      [this, driverDeadBand] {
                        return driverDeadBand(m_driverStick.GetRawAxis(
                            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS));
                      },
                      [this, driverDeadBand] {
                        return driverDeadBand(m_driverStick.GetRawAxis(
                            OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS));
                      }};
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
