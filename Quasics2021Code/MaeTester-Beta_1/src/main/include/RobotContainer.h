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
#include "VisionSettingsHelper.h"
#include "commands/ColorLights.h"
#include "commands/TankDrive.h"
#include "commands/TriggerDrivenShootingCommand.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/Shooter.h"

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// DEFINE this symbol to enable the pneumatics subsystem.  (This is not
// installed on Nike.)
#define ENABLE_PNEUMATICS

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
  void AddShootAndMoveTestsToSmartDashboard();
  void AddPneumaticsControlsToSmartDashboard();
  void AddExampleTrajectoryCommandsToSmartDashboard();
  void AddSampleMovementCommandsToSmartDashboard();

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
  void RunCommandWhenOperatorButtonIsHeld(int buttonId,
                                          frc2::Command* command);

  /// Utility function to help bind a command to a specific button on the
  /// driver's controller.
  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command* command);

  //////////////////////////////////////////////////////////////////
  // Utility functions for generating trajectory-based commands.
 private:
  /// Generates a command group to run the specified trajectory.
  frc2::SequentialCommandGroup* GenerateRamseteCommand(
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

  /// Generates a command group to run the trajectory stored in the specified
  /// file.
  frc2::SequentialCommandGroup* GenerateRamseteCommandFromPathFile(
      std::string filename, bool resetTelemetryAtStart);

  frc::TrajectoryConfig buildConfig();
  frc2::SequentialCommandGroup* createRams(frc::Trajectory trajectory,
                                           bool resetTelemetryAtStart);
  frc::Trajectory loadTraj(std::string jsonFile);

  //////////////////////////////////////////////////////////////////
  // Functions for Bounce Path challenge.
 private:
  /// Builds a command group for the Bounce Path challenge.
  frc2::SequentialCommandGroup* BuildBouncePathCommand();

  //////////////////////////////////////////////////////////////////
  // Functions for Galactic Search challenge.
 private:
  /// Returns true iff the RasPi can't identify the current Galactic Search
  /// variant based on the video from the camera.
  bool RecognizeError();

  /// Returns true iff the RasPi says the Galactic Search variant is for Path A
  /// options.
  bool RecognizePathA();

  /// Returns true iff the RasPi says the Galactic Search variant is for the
  /// Blue alliance.
  bool RecognizeBlueAlliance();

  /// Builds a command group for the GS challenge, taking the 4 paths to
  /// balls and the end zone from the specified files.  (Will optionally
  /// include the intake operation, running in parallel.)
  frc2::ParallelCommandGroup* BuildGalacticSearchPath(
      std::string jsonFile1, std::string jsonFile2, std::string jsonFile3,
      std::string jsonFile4, bool includeIntakeOperation = true);

  /// Builds a conditional command that handles GS Paths A/B for Blue alliance.
  frc2::ConditionalCommand* BuildBlueAlliancePaths();

  /// Builds a conditional command that handles GS Paths A/B for Red alliance.
  frc2::ConditionalCommand* BuildRedAlliancePaths();

  /// Builds a conditional command that picks GS handling for Blue or Red
  /// alliance.
  frc2::ConditionalCommand* ChooseWhichAlliance();

  /// Builds a conditional command that picks GS handling for any alternative
  /// signalled by the RasPi.
  frc2::ConditionalCommand* GalacticSearchAutoPath();

  //////////////////////////////////////////////////////////////////
  // Functions for simple "shoot and move" auto mode in Infinite Recharge.
 private:
  frc2::SequentialCommandGroup* BuildConveyorSeqeunceForAuto(
      units::second_t secondsToRunConveyor, units::second_t secondsToWait);

  frc2::ParallelRaceGroup* BuildConveyorAndShootingSequence(
      units::second_t secondsToRunConveyor, units::second_t secondsToWait,
      units::second_t timeForRunShooter);

  frc2::SequentialCommandGroup* BuildShootAndMoveSequence(
      units::second_t secondsToRunConveyor, units::second_t secondsToWait,
      units::second_t timeForRunShooter, double power, double amountToMove);

  //////////////////////////////////////////////////////////////////
  // Subsystems
 private:
  Drivebase drivebase;
  Shooter shooter;
  Intake intake;
  Lights lights;

#ifdef ENABLE_PNEUMATICS
  Pneumatics pneumatics;
#endif  // ENABLE_PNEUMATICS

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

  //////////////////////////////////////////////////////////////////
  // Stuff for 2021 approach to vision processing.
 private:
  /// Used to tune/store the Vision.py code's color range.
  VisionSettingsHelper m_visionSettingsHelper{
      VisionSettingsHelper::GetSuggestedRoboRioDirectory() +
      "visionSettings.dat"};

  /// Used to read the GS path identification provided by the RasPi.
  nt::NetworkTableEntry pathId;
};
