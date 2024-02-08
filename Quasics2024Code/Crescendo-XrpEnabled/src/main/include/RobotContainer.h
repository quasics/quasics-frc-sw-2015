// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <list>

#include "ConditionalCompileFlags.h"
#include "ConfigSettings.h"
#include "Constants.h"
#include "commands/SetRobotOdometry.h"
#include "subsystems/Climber.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"
#include "subsystems/Shooter.h"
#include "subsystems/Vision.h"
#include "utils/DeadBandEnforcer.h"

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

  // Configuration/set-up support functions.
 private:
  void allocateDriveBase();
  void setUpTankDrive();
  void setUpArcadeDrive();

  // Functions to bind commands to the driver/operator controllers.
 private:
  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command* command);

  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command* command);

  void ConfigureDriverControllerButtonBindings();
  void ConfigureOperatorControllerButtonBindings();

  // Functions to set up Smart Dashboard (auto mode, etc.).
 private:
  static frc2::Command* BuildNamedPrintCommand(std::string name,
                                               std::string text = "");
  static void AddNamedCommandToSelector(
      frc::SendableChooser<frc2::Command*>& selector, std::string name,
      std::string text = "");
  static void AddingNamedStartingPositionsToSelector(
      frc::SendableChooser<frc2::Command*>& selector);
  static void AddingNamedOverallOperationsToSelector(
      frc::SendableChooser<frc2::Command*>& selector);
  static void AddingNamedScoreDestinationsToSelector(
      frc::SendableChooser<frc2::Command*>& selector1,
      frc::SendableChooser<frc2::Command*>& selector2);

  void AddTestButtonsOnSmartDashboard();
  void AddButtonToSmartDashboardTestingRetainedCommands();

  // Driving support functions
 private:
  enum class DriveMode { eNormal, eSwitched };
  void setDriveMode(DriveMode mode);
  double GetDriveSpeedScalingFactor();

  // Auto mode functions
 private:
  // AUTOS
  void AddAutoSelectionsToSmartDashboard();
  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotOverallOperationToSmartDashboard();
  void AddScoreDestinationsToSmartDashboard();

  // Data members of the class (subsystems, persistent commands, etc.)
 private:
  ConfigSettings m_configSettings;
  frc::Joystick m_driverController{0};
  frc::XboxController m_operatorController{1};

  // These are commands generated "on the fly" by functions like
  // GetCommandForTrajectory(), which we're attaching to things like buttons on
  // the smart dashboard, and thus need to outlive the function where they were
  // created.
  std::list<frc2::CommandPtr> retainedCommands;

  std::unique_ptr<IDrivebase> m_drivebase;

  const DeadBandEnforcer m_joystickDeadbandEnforcer{0.03};

  frc::SlewRateLimiter<units::scalar> m_leftSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};
  frc::SlewRateLimiter<units::scalar> m_rightSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};

#ifdef ENABLE_VISION_SUBSYSTEM
  // Intentionally leaking for now....
  Vision* m_vision{new Vision};
#endif  // ENABLE_VISION_SUBSYSTEM

  Shooter m_shooter;
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  Shooter m_shooter;
  Climber m_climber;
  IntakeDeployment m_intakeDeployment;
  IntakeRoller m_intakeRoller;
#endif  // ENABLE_FULL_ROBOT_FUNCTIONALITY

  frc::SendableChooser<frc2::Command*> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command*> m_OverallAutonomousOptions;
  frc::SendableChooser<frc2::Command*> m_Score2DestAutonomousOptions;
  frc::SendableChooser<frc2::Command*> m_Score3DestAutonomousOptions;
};
