// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/CommandXboxController.h>

#include "ConfigSettings.h"
#include "Constants.h"
#include "subsystems/Drivebase.h"
#include "subsystems/FloorEjection.h"
#include "subsystems/IntakeClamp.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"
#include "subsystems/Lighting.h"
#include "subsystems/PhotonLibVision.h"

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

  frc2::Command *GetAutonomousCommand();

  void RunCommandWhileDriverButtonIsHeld(int logitechButtonId,
                                         frc2::Command *command);
  void RunCommandWhenDriverButtonIsPressed(int logitechButtonId,
                                           frc2::Command *command);
  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command *command);

  double GetDriveSpeedScalingFactor();

  void ConfigureControllerButtonBindings();
  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotSequenceSelectorToSmartDashboard();

  void setInverted(bool invert);

  void AddTestButtonsToSmartDashboard();

  frc2::SequentialCommandGroup *TESTCOMMAND();

 private:
  // Driver's controller.
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // Operator's controller.
  frc::XboxController m_operatorController{1};

  // The robot's subsystems are defined here...
  Drivebase m_drivebase;
  IntakeClamp m_intakeClamp;
  IntakeRoller m_intakeRoller;
  IntakeDeployment m_intakeDeployment;
  PhotonLibVision m_photonLibVision;
  FloorEjection m_floorEjection;
  Lighting m_lighting;

  // Settings accessed across multiple commands
  ConfigSettings m_configSettings;

  // Supporting autonomous mode command selection/building.
  frc::SendableChooser<frc2::Command *> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command *> m_RobotSequenceAutonomousOptions;

  // Supporting tank drive-related stuff.
  bool isInverted = true;
  frc::SlewRateLimiter<units::scalar> m_leftSlewRateLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSlewRateLimiter;
};
