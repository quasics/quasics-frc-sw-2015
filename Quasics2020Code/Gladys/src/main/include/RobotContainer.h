/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/ExampleCommand.h"

#include "subsystems/CommandPanel.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Exhaust.h"
#include "subsystems/Climber.h"
#include <frc/Joystick.h>

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
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;
  Drivebase drivebase;
  frc2::InstantCommand enableTurboMode {[this] {drivebase.EnableTurboMode();}, {}};
  frc2::InstantCommand disableTurboMode {[this] {drivebase.DisableTurboMode();}, {}};
  frc2::InstantCommand frontIsForward {[this] {drivebase.SwitchFace();}, {}};
  Intake intake;
  Exhaust exhaust;
  Climber climber;
  frc::Joystick driverJoystick{0};

  void ConfigureButtonBindings();

};
