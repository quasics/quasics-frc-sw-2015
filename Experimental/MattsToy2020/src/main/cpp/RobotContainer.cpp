/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include "commands/TankDriveCommand.h"

const double kDeadBandSize = 0.015;
const double kPowerScalingFactor = 0.45;

// Software dead-band handling on readings from the driver's control.
inline double applyDeadBandAdjustment(double stickValue) {
  if (std::abs(stickValue) < kDeadBandSize) {
    return 0;
  }
  return stickValue;
}

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
  m_driveBase.SetDefaultCommand(TankDriveCommand(
      &m_driveBase,
      [this] {
        return kPowerScalingFactor *
               applyDeadBandAdjustment(
                   m_driverController.GetY(frc::GenericHID::kLeftHand));
      },
      [this] {
        return kPowerScalingFactor *
               applyDeadBandAdjustment(
                   m_driverController.GetX(frc::GenericHID::kRightHand));
      }));

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
