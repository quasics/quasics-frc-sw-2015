// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/AbstractDriveBase.h"
#include "subsystems/ExampleSubsystem.h"

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

 private:
  /**
   * Creates the drive base object for the hardware (real or otherwise) that
   * we're actually "talking to".
   */
  void allocateDriveBase();

  /** Sets up tank drive as the default command for our drive base. */
  void setUpTankDrive();

 private:
  frc::Joystick m_controller{0};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;

  // We'll allocate our drive base with "new", so that we can pick the "flavor"
  // that we need for what we're trying to talk to.
  std::unique_ptr<AbstractDriveBase> m_drivebase;
};
