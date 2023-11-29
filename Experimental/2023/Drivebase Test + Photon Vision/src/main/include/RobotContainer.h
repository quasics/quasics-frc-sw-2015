// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>

#include "Constants.h"
#include "subsystems/Drivebase.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
    
  //frc2::CommandPtr DriveInALine(units::meter_t distance, bool reversed, Drivebase *drivebase);

  frc2::CommandPtr DriveInALineUsingAprilTags(units::meter_t distance);

    frc2::SequentialCommandGroup *DriveStraightLineAprilTag(
      units::meter_t distance);

  frc2::CommandPtr TestManualBuild();

  void AddTestButtonsOnSmartDashboard();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems are defined here...
  Drivebase m_drivebase;

  void ConfigureBindings();
};
