// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <unistd.h>

#include "../../../../Common2021/DeadBandEnforcer.h"
#include "../../../../Common2021/SpeedScaler.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "../../../../Common2021/TrajectoryCommandGenerator.h"
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
        return -scaler(deadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::LeftYAxis)));
      },
      [this, deadBand, scaler] {
        return -scaler(deadBand(m_driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis)));
      });
  m_driveBase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureButtonBindings();

  ConfigureShuffleboard();
}

void RobotContainer::ConfigureShuffleboard() {
  // Configure trajectory generation.  (Note that this could be
  // embedded in the RobotContainer -- or elsewhere -- and reused
  // as needed, including tweaks to max speed/acceleration.)
  TrajectoryCommandGenerator generator(
      // Drive base being controlled
      &m_driveBase,
      // Drive profile data
      {RobotData::DriveConstants::ksVolts,
       RobotData::DriveConstants::kvVoltSecondsPerMeter,
       RobotData::DriveConstants::kaVoltSecondsSquaredPerMeter},
      // PID configuration values
      {RobotData::DriveConstants::kPDriveVel,
       RobotData::DriveConstants::kIDriveVel,
       RobotData::DriveConstants::kDDriveVel},
      // Speed profile
      {RobotData::PathFollowingLimits::kMaxSpeed,
       RobotData::PathFollowingLimits::kMaxAcceleration});

  auto sampleTrajectory_3mForward = generator.GenerateCommand(
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m),
                                      frc::Translation2d(2_m, 0_m)},
      // Ending pose
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true);

  m_driveBase.AddToShuffleboard("3m forward", sampleTrajectory_3mForward);

  auto sampleTrajectory_sCurve = generator.GenerateCommand(
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{
          frc::Translation2d(1_m, 0.5_m),
          frc::Translation2d(2_m, -0.5_m),
      },
      // Ending pose
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true);

  m_driveBase.AddToShuffleboard("S-curve", sampleTrajectory_sCurve);

  auto sampleTrajectory_figureEight = generator.GenerateCommand(
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{
          frc::Translation2d(1_m, 1_m),
          frc::Translation2d(2_m, -1_m),
          frc::Translation2d(3_m, 0_m),
          frc::Translation2d(2_m, 1_m),
          frc::Translation2d(1_m, -1_m),
      },
      // Ending pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), true);

  m_driveBase.AddToShuffleboard("Figure-8", sampleTrajectory_figureEight);

  m_driveBase.AddToShuffleboard("Turn to target", &turnToTarget);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &turnToTarget;
}
