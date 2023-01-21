// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TankDrive.h"
#include "Constants.h"

RobotContainer::RobotContainer() {

  TankDrive tankDrive {
    &m_driveBase,
    [this] {

      // Figure out scaling for the current driving mode
      const double scalingFactor = GetDriveSpeedScalingFactor();

      double joystickValue;

      if(!isInverted) {
        joystickValue = -1 * scalingFactor *
                              m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        
      }
     else {
      joystickValue = +1 * scalingFactor *
                            m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
     }
     return joystickValue;
    },
    [this] {

      const double scalingFactor = GetDriveSpeedScalingFactor();

      double joystickValue;

      if(!isInverted) {
        joystickValue = -1 * scalingFactor *
                              m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        
      }
     else {
      joystickValue = +1 * scalingFactor *
                            m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
     }
     return joystickValue;
    }
  };
  
  

  m_driveBase.SetDefaultCommand(tankDrive);

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);

  if (isTurbo) {
    return RobotValues::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotValues::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotValues::NORMAL_MODE_SPEED_SCALING;
  }
}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
