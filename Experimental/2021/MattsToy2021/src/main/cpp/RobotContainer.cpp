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
  auto sampleTrajectorySequence = GenerateRamseteCommand(
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m),
                                      frc::Translation2d(2_m, 0_m)},
      // Ending pose
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true);

  // TODO(mjh): Move this to a sub-tab on Shuffleboard, vs directly on
  // SmartDashboard
  frc::SmartDashboard::PutData("3m forward", sampleTrajectorySequence);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &turnToTarget;
}

frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {
#if 0
  TrajectoryCommandGenerator generator(&m_driveBase);
  return generator.GenerateCommand(start, interiorWaypoints, end,
                                   resetTelemetryAtStart);
#else
  using namespace RobotData::DriveConstants;
  using namespace RobotData::PathFollowingLimits;

  const frc::DifferentialDriveKinematics kDriveKinematics{
      m_driveBase.GetTrackWidth()};

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  config.SetKinematics(kDriveKinematics);

  config.AddConstraint(voltageConstraints);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_driveBase.GetPose(); },
      frc::RamseteController{kRamseteB, kRamseteZeta}, feedForward,
      kDriveKinematics, [this]() { return m_driveBase.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) {
        m_driveBase.TankDriveVolts(left, right);
      },
      {&m_driveBase});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand(
          [this, resetTelemetryAtStart, exampleTrajectory] {
            if (resetTelemetryAtStart) {
              m_driveBase.ResetOdometry(exampleTrajectory.InitialPose());
            }
          },
          {&m_driveBase}),
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_driveBase.TankDriveVolts(0_V, 0_V); },
                           {&m_driveBase}));
#endif
}
