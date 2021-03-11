// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <networktables/NetworkTableEntry.h>

#include <vector>

#include "../../../../Common2021/VisionSettingsHelper.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/OnBoardIO.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

 private:
  void ConfigureButtonBindings();
  void ConfigureDrivingCommand();
  void ConfigureAutonomousSelection();
  void EnableTankDrive();
  void EnableArcadeDrive();

  enum TrajectoryExample {
    StraightLineTrajectory,
    S_CurveTrajectory,
    FigureEightTrajectory
  };

  /**
   * Generates a sample command to follow a simple trajectory.
   *
   * @param resetTelemetryAtStart
   *     if true, the command will (at its initiation) reset the drive
   *     telemetry, allowing it to start following the trajectory using
   *     its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   */
  frc2::SequentialCommandGroup* GenerateRamseteCommand(
      bool resetTelemetryAtStart,
      TrajectoryExample example = StraightLineTrajectory);

  /**
   * Generates a command to follow the specified trajectory.
   * @param start  starting position/orientation (relative to drive telemetry
   *               data)
   * @param interiorWaypoints
   *               positions/orientations to be passed through while
   *               proceeding from "start" to "end"
   * @param end    ending position/orientation (relative to drive telemetry
   *               data)
   * @param resetTelemetryAtStart
   *     if true, the command will (at its initiation) reset the drive
   *     telemetry, allowing it to start following the trajectory using
   *     its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   */
  frc2::SequentialCommandGroup* GenerateRamseteCommand(
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

 private:
  // Helper component to manage settings for vision processing.
  VisionSettingsHelper m_helper{
      VisionSettingsHelper::GetSuggestedRomiDirectory() + "visionSettings.dat"};

  // Assumes a gamepad is plugged into channnel 0.
  frc::Joystick m_controller{0};

  // The robot's subsystems and some key commands.
  Drivetrain m_drive;
  OnBoardIO m_onboardIO{OnBoardIO::ChannelMode::INPUT,
                        OnBoardIO::ChannelMode::INPUT};

  std::vector<std::shared_ptr<frc2::Command>> m_autoModeOptions;
  frc::SendableChooser<frc2::Command*> m_autonomousChooser;
  nt::NetworkTableEntry m_autoModeSelection;
};
