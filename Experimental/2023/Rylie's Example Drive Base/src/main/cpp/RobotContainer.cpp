// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc/Joystick.h>

#include "commands/ArcadeDrive.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() : m_leftSlewRateLimiter{1 / 1_s},
      m_rightSlewRateLimiter{1 / 1_s}{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  SetDefaultArcadeDrive();
}

void::RobotContainer::SetDefaultTankDrive(){
  TankDrive tankDrive{
    &m_drivebase,
    [this] {
      double joystickValue;
      joystickValue = -1 * m_driverStick.GetRawAxis(OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
      return m_leftSlewRateLimiter.Calculate(joystickValue);
    },
    [this] {
      double joystickValue;
      joystickValue = -1 * m_driverStick.GetRawAxis(OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
      return m_rightSlewRateLimiter.Calculate(joystickValue);
    }
  };
  m_drivebase.SetDefaultCommand(tankDrive);
}

void RobotContainer::SetDefaultArcadeDrive(){
  ArcadeDrive arcadeDrive{
 &m_drivebase,
    [this] {
      double joystickValue;
      joystickValue = -1 * m_driverStick.GetRawAxis(OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
      return m_leftSlewRateLimiter.Calculate(joystickValue);
    },
    [this] {
      double joystickValue;
      joystickValue = -1 * m_driverStick.GetRawAxis(OperatorInterface::LogitechGamePad::RIGHT_X_AXIS);
      return m_rightSlewRateLimiter.Calculate(joystickValue);
    }
};
m_drivebase.SetDefaultCommand(arcadeDrive);
}

void RobotContainer::ConfigureBindings() {

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
