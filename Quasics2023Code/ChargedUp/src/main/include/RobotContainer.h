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
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/FloorEjection.h"
#include "subsystems/IntakeClamp.h"
#include "subsystems/IntakeDeployment.h"
#include "subsystems/IntakeRoller.h"
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

  void RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                        frc2::Command *command);

  void RunCommandWhenOperatorButtonIsHeld(int buttonId, frc2::Command *command);

  double GetDriveSpeedScalingFactor();

  void ConfigureControllerButtonBindings();
  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotSequenceSelectorToSmartDashboard();

  void setInverted(bool invert);

 private:
  frc2::Command *GTFODOCK(std::string teamAndPosName);

  frc2::Command *moveToDefenseAgainstScoringWall(std::string teamAndPosName);

  frc2::Command *moveToDefenseAgainstOuterWall(std::string teamAndPosName);

  frc2::Command *JustCharge(std::string teamAndPosName);

  frc2::Command *ScoreAndLeave(std::string teamAndPosName);

  frc2::Command *ScoreThenCharge(std::string teamAndPosName);

  frc2::Command *ScoreThenEndNearGamePieceCommand(std::string teamAndPosName);
  frc2::Command *DropGamePieceThenGTFOCommand(std::string teamAndPosName);

  frc2::Command *DropGamePieceThenChargeCommand(std::string teamAndPosName);

  frc2::Command *ScoreGTFOThenCharge(std::string teamAndPosName);

  frc2::SequentialCommandGroup *DropGamePieceHelperCommand();

  frc2::SequentialCommandGroup *ClampScoreGamePieceHelperCommand();

  frc2::SequentialCommandGroup *RollerScoreGamePieceHelperCommand();

  frc2::SequentialCommandGroup *GetScoreSequenceFromStartingPoint();

 private:
  // CODE_REVIEW: Consolidate to just one driver joystick/controller, please!!!!
  // (mjh)
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // Removed this
  // frc2::CommandXboxController m_driverController{
  //     OperatorConstants::kDriverControllerPort};

  // frc::Joystick driverJoystick{0};

  frc::XboxController m_operatorController{1};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Drivebase m_drivebase;
  IntakeClamp m_intakeClamp;
  IntakeRoller m_intakeRoller;
  IntakeDeployment m_intakeDeployment;
  PhotonLibVision m_photonLibVision;
  FloorEjection m_floorEjection;
  ConfigSettings *m_configSettings;

  frc::SendableChooser<frc2::Command *> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command *> m_RobotSequenceAutonomousOptions;

  void ConfigureBindings();
  void AddTestButtonsToSmartDashboard();

  bool isInverted = true;
  frc::SlewRateLimiter<units::scalar> m_leftSpeedLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSpeedLimiter;
};
