// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/command/Command.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <networktables/NetworkTableEntry.h>

#include "SpeedScaler.h"  // TODO(scott): Replace this example.
#include "commands/ColorLights.h"
#include "commands/TankDrive.h"
#include "commands/TriggerDrivenShootingCommand.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"
#include "subsystems/Shooter.h"

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// Conditional compilation flags end here.
///////////////////////////////////////////////////////////////////////////////

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

  /// Used by the Robot class to get the command to be run in autonomous mode.
  frc2::Command* GetAutonomousCommand();

 private:
  /// Builds the (fairly complex) tank drive command, but doesn't install it
  /// on the subsystem.
  std::unique_ptr<TankDrive> BuildTankDriveCommand();

  /// Builds the (somewhat complex) shooting command, but doesn't install it
  /// on the subsystem.
  std::unique_ptr<TriggerDrivenShootingCommand> BuildShootingCommand();

  /// Configures the default commands for various subsystems.
  void InstallDefaultCommands();

  /// Configures the options shown in the selector for autonomous mode
  /// operations, and puts the selector widget on the smart dashboard.
  void ConfigureAutoSelection();

  // The following functions each add buttons for particular sets of
  // functionality to the smart dashboard.  They may be called from the
  // ConfigureSmartDashboard() function.
  void AddLightingButtonsToSmartDashboard();
  void AddShooterAngleControlsToSmartDashboard();
  void AddShooterSpeedControlsToSmartDashboard();

  /// Adds various buttons shown on the smart dashboard.
  void ConfigureSmartDashboard();

  // Binds the buttons on the driver and operator controllers to specific
  // commands.
  void ConfigureControllerButtonBindings();
  void ConfigureDriverButtonBindings();
  void ConfigureOperatorButtonBindings();

  /// Utility function to provide "deadband" regions to Joysticks (i.e., if it's
  /// within a defined range from 0, then treat it *as* 0).
  static double deadband(double num);

  /// Utility function to help bind a command to a specific button on the
  /// operator's controller.
  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command* command);

  /// Utility function to help bind a command to a specific button on the
  /// driver's controller.
  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command* command);

  //////////////////////////////////////////////////////////////////
  // Subsystems
 private:
  Drivebase drivebase;
  Shooter shooter;
  Intake intake;
  Lights lights;

  //////////////////////////////////////////////////////////////////
  // Controllers
 private:
  frc::Joystick driverJoystick{0};
  frc::XboxController operatorController{1};

  //////////////////////////////////////////////////////////////////
  // Commands and related "stuff"
 private:
  /// Default command for lights: solid green (for Quasics!).
  ColorLights m_defaultLightingCommand{&lights, 0, 255, 0};

  /// Widget shown on the smart dashboard, allowing drive team to select
  /// a command for execution during autonomous mode.
  frc::SendableChooser<frc2::Command*> m_autoChooser;
};
