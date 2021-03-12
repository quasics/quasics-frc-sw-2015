// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "../../../../Common2021/TurnToTargetCommand.h"
#include "../../../../Common2021/VisionSettingsHelper.h"
#include "commands/ExampleCommand.h"
#include "subsystems/DriveBase.h"
#include "subsystems/ExampleSubsystem.h"

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

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureButtonBindings();

  void ConfigureShuffleboard();

  frc2::SequentialCommandGroup* GenerateRamseteCommand(
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;

  VisionSettingsHelper m_helper{"/home/lvuser/visionSettings.dat"};

  // Assumes a gamepad plugged into channnel 0
  frc::Joystick m_driverJoystick{0};

  DriveBase m_driveBase;
  TurnToTargetCommand turnToTarget{&m_driveBase, 0.2};
};
