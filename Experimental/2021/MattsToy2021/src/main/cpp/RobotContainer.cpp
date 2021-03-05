// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <unistd.h>

#include "../../../../Common2021/DeadBandEnforcer.h"
#include "../../../../Common2021/SpeedScaler.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "Constants.h"

constexpr double kMaxTurtleSpeed = 0.4;
constexpr double kMaxNormalSpeed = 0.6;
constexpr double kMaxTurboSpeed = 0.75;

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  std::vector<char> buffer(4096);
  if (getcwd(&buffer[0], buffer.size()) != NULL) {
    std::cerr << "Current directory is: " << &buffer[0] << std::endl;
  } else {
    std::cerr << "Failed to get current directory" << std::endl;
  }

  m_helper.InstallSliders();

  // Initialize all of your commands and subsystems here
  DeadBandEnforcer deadBand(OIConstants::DeadBand_LowValue,
                            OIConstants::DeadBand_HighValue);
  DeadBandEnforcer scalingDeadBand(0.25);
  SpeedScaler scaler(
      [this, scalingDeadBand] {
        constexpr int turtleTrigger =
            OIConstants::LogitechGamePad::LeftTriggerAxis;
        constexpr int turboTrigger =
            OIConstants::LogitechGamePad::RightTriggerAxis;
        if (scalingDeadBand(m_driverJoystick.GetRawAxis(turtleTrigger)) > 0)
          return SpeedScaler::Mode::Turtle;
        else if (scalingDeadBand(m_driverJoystick.GetRawAxis(turboTrigger)) > 0)
          return SpeedScaler::Mode::Turbo;
        else
          return SpeedScaler::Mode::Normal;
      },
      kMaxNormalSpeed, kMaxTurtleSpeed, kMaxTurboSpeed);
  TeleopTankDrive tankDrive(
      &m_driveBase,
      [this, deadBand, scaler] {
        return scaler(deadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis)));
      },
      [this, deadBand, scaler] {
        return scaler(deadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::LeftYAxis)));
      });
  m_driveBase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &turnToTarget;
}
