// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "commands/ExampleCommand.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Shooter.h"

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
  Shooter shooter;

  // Controllers
  frc::Joystick driverJoystick{0};
  frc::XboxController operatorController{1};

  
  void ConfigureSmartDashboard();
  void ConfigureButtonBindings();
  void ConfigureDriverButtonBindings();
  void ConfigureOperatorButtonBindings();
  void RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button buttonId, frc2::Command* command);
  double deadband(double num);
};
