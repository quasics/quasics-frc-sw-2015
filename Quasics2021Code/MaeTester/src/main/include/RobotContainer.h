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
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/Shooter.h"

// #define ENABLE_PNEUMATICS

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
  TankDrive* BuildTankDriveCommand();
  ColorLights* BuildColorLightsCommand();
  void ConfigureTankDrive();
  void ConfigureLights();
  void ConfigureAutoSelection();
  void ConfigureSmartDashboard();
  void ConfigureButtonBindings();
  void ConfigureDriverButtonBindings();
  void ConfigureOperatorButtonBindings();

  static double deadband(double num);

  void RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button buttonId,
                                          frc2::Command* command);

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
  /// Returns true iff the RasPi can't identify the GS variant.
  bool RecognizeError();
  /// Returns true iff the RasPi says the GS variant is for Path A options.
  bool RecognizePathA();
  /// Returns true iff the RasPi says the GS variant is for the Blue alliance.
  bool RecognizeBlueAlliance();

  /// Builds a command group for the GS challenge, taking the 4 paths to
  /// balls and the end zone from the specified files.  (Will optionally
  /// include the intake operation, running in parallel.)
  frc2::ParallelCommandGroup* BuildGalacticSearchPath(std::string jsonFile1,
                                                        std::string jsonFile2,
                                                        std::string jsonFile3,
                                                        std::string jsonFile4,
                                                        bool includeIntakeOperation = true);

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
  // Data/member objects: subsystems, helpers, etc.
 private:
  Drivebase drivebase;
  Shooter shooter;
  Intake intake;
  Lights lights;
#ifdef ENABLE_PNEUMATICS
  Pneumatics pneumatics;
#endif  // ENABLE_PNEUMATICS

  frc::SendableChooser<frc2::Command*> m_autoChooser;

  // Controllers
  frc::Joystick driverJoystick{0};
  frc::XboxController operatorController{1};

  /// Used to tune/store the Vision.py code's color range.
  VisionSettingsHelper m_visionSettingsHelper{
      VisionSettingsHelper::GetSuggestedRoboRioDirectory() +
      "visionSettings.dat"};

  /// Used to read the GS path identification provided by the RasPi.
  nt::NetworkTableEntry pathId;
};
