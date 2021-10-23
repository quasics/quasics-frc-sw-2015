// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <unistd.h>

#include "Common2021/DeadBandEnforcer.h"
#include "Common2021/DriveDistance.h"
#include "Common2021/SpeedScaler.h"
#include "Common2021/TeleopTankDrive.h"
#include "Constants.h"

constexpr double kMaxTurtleSpeed = 0.4;
constexpr double kMaxNormalSpeed = 0.6;
constexpr double kMaxTurboSpeed = 0.75;

RobotContainer::RobotContainer()
    : m_trajectoryGenerator(
          // Drive base being controlled
          &m_driveBase,
          // Drive profile data
          {RobotData::DriveConstants::ksVolts,
           RobotData::DriveConstants::kvVoltSecondsPerMeter,
           RobotData::DriveConstants::kaVoltSecondsSquaredPerMeter},
          // PID configuration values
          {RobotData::DriveConstants::kPDriveVel,
           RobotData::DriveConstants::kIDriveVel,
           RobotData::DriveConstants::kDDriveVel}) {
  // Set up access to results from vision processor on RasPi
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable(NetworkTableNames::kVisionTable);
  m_imageWidth = table->GetEntry(NetworkTableNames::kImageWidthEntry);
  m_imageHeight = table->GetEntry(NetworkTableNames::kImageHeightEntry);
  m_targetList_x = table->GetEntry(NetworkTableNames::kTargetXEntry);
  m_targetList_y = table->GetEntry(NetworkTableNames::kTargetYEntry);
  m_targetList_top =
      table->GetEntry(NetworkTableNames::kTargetAllTargetsTopEntry);
  m_targetList_left =
      table->GetEntry(NetworkTableNames::kTargetAllTargetsLeftEntry);
  m_targetList_width =
      table->GetEntry(NetworkTableNames::kTargetAllTargetsWidthEntry);
  m_targetList_height =
      table->GetEntry(NetworkTableNames::kTargetAllTargetsHeightEntry);

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

  ConfigureAutonomousSelection();
}

Rectangle RobotContainer::GetVisionSizing() {
  return Rectangle{0, 0, m_imageWidth.GetDouble(0), m_imageHeight.GetDouble(0)};
}

bool RobotContainer::HavePrimaryTarget() {
  return !m_targetList_x.GetDoubleArray({}).empty();
}

unsigned int RobotContainer::GetNumPossibleTargets() {
  return m_targetList_top.GetDoubleArray({}).size();
}

std::vector<Rectangle> RobotContainer::GetPossibleTargetRects() {
  std::vector<Rectangle> results;
  auto tops = m_targetList_top.GetDoubleArray({});
  auto lefts = m_targetList_left.GetDoubleArray({});
  auto widths = m_targetList_width.GetDoubleArray({});
  auto heights = m_targetList_height.GetDoubleArray({});
  if (tops.size() != lefts.size() || widths.size() != heights.size() ||
      tops.size() != widths.size()) {
    /* TODO(mhealy): Log the error. */
    return results;
  }

  // Add each of the rectangles to the results.
  for (unsigned int i = 0; i < tops.size(); ++i) {
    results.push_back({tops[i], lefts[i], widths[i], heights[i]});
  }
  return results;
}

void RobotContainer::ConfigureAutonomousSelection() {
  // Note: I'm working with a SendableChooser here, which unfortunately only
  // works when you're using the actual DriverStation software and
  // SmartDashboard (or Shuffleboard); this means that this will *only* be
  // able to work when we've got a Windows box handy, and not when I'm working
  // solely on my Mac.
  //
  // However, it'll provide a clean example for the team, which is good.  (But
  // that's also one of the reasons that I'm putting in a "do nothing" command
  // as the default.)
  m_autonomousChooser.SetDefaultOption(
      "Do nothing",
      new frc2::PrintCommand("Explicitly doing nothing for Auto mode...."));

  m_autonomousChooser.AddOption(
      "Move 3m forward (traj.)",
      m_trajectoryGenerator.GenerateCommand(
          // Speed profile
          {RobotData::PathFollowingLimits::kMaxSpeed,
           RobotData::PathFollowingLimits::kMaxAcceleration},
          // Starting pose
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          // Interior waypoints
          std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m),
                                          frc::Translation2d(2_m, 0_m)},
          // Ending pose
          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));

  m_autonomousChooser.AddOption("Move 4m forward",
                                new DriveDistance(&m_driveBase, 4_m, 0.5));

  // Example of building a sequential command group from pieces.
  //
  // (All of the "std::move" stuff is because the compiler is enforcing the
  // contract that only 1 thing "knows about" the pointed-to commands, which
  // lets the SequentialCommandGroup make certain assumptions about them.)
  {
    std::vector<std::unique_ptr<frc2::Command>> sequence;
    sequence.push_back(
        std::move(std::make_unique<DriveDistance>(&m_driveBase, 2_m, 0.5)));
    sequence.push_back(
        std::move(std::make_unique<DriveDistance>(&m_driveBase, -2_m, 0.5)));
    m_autonomousChooser.AddOption(
        "2m forward and return",
        new frc2::SequentialCommandGroup(std::move(sequence)));
  }

  // Put the SendableChooser on the Smart Dashboard for the driver's
  // station.
  frc::SmartDashboard::PutData("Auto mode", &m_autonomousChooser);
}

frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand(
    const std::string jsonFileName, bool resetTelemetryAtStart) {
  return m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
      jsonFileName, resetTelemetryAtStart
                        ? TrajectoryCommandGenerator::ResetTelemetryAtStart
                        : TrajectoryCommandGenerator::UseExistingTelemetry);
}

void RobotContainer::ConfigureShuffleboard() {
  TrajectoryCommandGenerator::SpeedProfile speedProfile = {
      RobotData::PathFollowingLimits::kMaxSpeed,
      RobotData::PathFollowingLimits::kMaxAcceleration};
  auto sampleTrajectory_3mForward = m_trajectoryGenerator.GenerateCommand(
      // Speed profile
      speedProfile,
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m),
                                      frc::Translation2d(2_m, 0_m)},
      // Ending pose
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      TrajectoryCommandGenerator::ResetTelemetryAtStart);

  m_driveBase.AddToShuffleboard("3m forward", sampleTrajectory_3mForward);

  auto sampleTrajectory_sCurve = m_trajectoryGenerator.GenerateCommand(
      // Speed profile
      speedProfile,
      // Starting pose
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Interior waypoints
      std::vector<frc::Translation2d>{
          frc::Translation2d(1_m, 0.5_m),
          frc::Translation2d(2_m, -0.5_m),
      },
      // Ending pose
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      TrajectoryCommandGenerator::ResetTelemetryAtStart);

  m_driveBase.AddToShuffleboard("S-curve", sampleTrajectory_sCurve);

  auto sampleTrajectory_figureEight = m_trajectoryGenerator.GenerateCommand(
      // Speed profile
      speedProfile,
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
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      TrajectoryCommandGenerator::ResetTelemetryAtStart);

  m_driveBase.AddToShuffleboard("Figure-8", sampleTrajectory_figureEight);

  m_driveBase.AddToShuffleboard("Turn to target", &turnToTarget);

  frc::SmartDashboard::PutData(
      "Target Conditional",
      new frc2::ConditionalCommand(frc2::PrintCommand("No targets seen"),
                                   frc2::PrintCommand("Targets spotted"),
                                   [this] { return HavePrimaryTarget(); }));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autonomousChooser.GetSelected();
}
