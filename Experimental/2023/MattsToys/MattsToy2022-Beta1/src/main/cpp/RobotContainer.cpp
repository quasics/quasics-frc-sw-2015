// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"
#include "commands/RainbowLighting.h"
#include "commands/TankDrive.h"
#include "utils/DeadBandEnforcer.h"

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
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
