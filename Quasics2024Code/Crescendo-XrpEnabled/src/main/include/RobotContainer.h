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
#include "Constants.h"
#include "commands/SetRobotOdometry.h"
#include "subsystems/Climber.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"
#include "subsystems/Shooter.h"
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

  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command* command);

  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command* command);

 private:
  void AddTestButtonsOnSmartDashboard();

  void AddButtonToSmartDashboardTestingRetainedCommands();
  // Replace with CommandPS4Controller or CommandJoystick if needed

  void allocateDriveBase();

  void setUpTankDrive();

  void setUpArcadeDrive();

 private:
  // AUTOS

  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotSequenceSelectorToSmartDashboard();
  frc2::CommandPtr testPathSequence();
  frc2::CommandPtr backwardForwardTest();
  frc2::CommandPtr backwardTest();

  double GetDriveSpeedScalingFactor();
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY

  void ConfigureDriverControllerButtonBindings();

  void ConfigureOperatorControllerButtonBindings();
#endif

  // The robot's subsystems are defined here...
  std::unique_ptr<IDrivebase> m_drivebase;
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  Shooter m_shooter;
  Climber m_climber;
  IntakeDeployment m_intakeDeployment;
  IntakeRoller m_intakeRoller;
#endif

  frc::Joystick m_driverController{0};
  frc::XboxController m_operatorController{1};

  // These are commands generated "on the fly" by functions like
  // GetCommandForTrajectory(), which we're attaching to things like buttons on
  // the smart dashboard, and thus need to outlive the function where they were
  // created.
  std::list<frc2::CommandPtr> retainedCommands;

  // void ConfigureBindings();

  const DeadBandEnforcer m_joystickDeadbandEnforcer{0.03};

  frc::SlewRateLimiter<units::scalar> m_leftSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};
  frc::SlewRateLimiter<units::scalar> m_rightSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};

  frc::SendableChooser<frc2::Command*> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command*> m_RobotSequenceAutonomousOptions;
};
