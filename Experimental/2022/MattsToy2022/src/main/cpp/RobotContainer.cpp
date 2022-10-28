// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"
#include "commands/RainbowLighting.h"
#include "commands/TankDrive.h"
#include "commands/MoveInALine.h"
#include "commands/rotate.h"

#include "utils/DeadBandEnforcer.h"

#include <frc/smartdashboard/SmartDashboard.h>

using Rate_t = frc::SlewRateLimiter<units::scalar>::Rate_t;

RobotContainer::RobotContainer()
    : m_leftSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT},
      m_rightSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT} {
  DeadBandEnforcer driverDeadBand{Deadbands::DRIVING};
  TankDrive tankDrive{
      &m_driveBase,
      [this, driverDeadBand] {
        const double currentLeftValue = driverDeadBand(m_driverStick.GetRawAxis(
            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS));
        return m_leftSpeedLimiter.Calculate(currentLeftValue);
      },
      [this, driverDeadBand] {
        const double currentRightValue =
            driverDeadBand(m_driverStick.GetRawAxis(
                OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS));
        return m_rightSpeedLimiter.Calculate(currentRightValue);
      }};
  m_driveBase.SetDefaultCommand(tankDrive);

  RainbowLighting rainbowLighting(&m_lighting, 0.1_s);
  m_lighting.SetDefaultCommand(rainbowLighting);

  // Configure the button bindings
  ConfigureButtonBindings();

  frc::SmartDashboard::PutData(".5m @ 40%", new MoveInALine(&m_driveBase, 0.5_m, .40));
  frc::SmartDashboard::PutData("1m @ 60%", new MoveInALine(&m_driveBase, 1_m, .60));
  frc::SmartDashboard::PutData("turn 90", new rotate(&m_driveBase, 90_m, .10));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
