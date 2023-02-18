// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Drivebase.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/IntakeClamp.h"
#include "subsystems/IntakeDeployment.h"

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

  double GetDriveSpeedScalingFactor();

  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotSequenceSelectorToSmartDashboard();

  void setInverted(bool invert);

 private:
  static frc2::Command *GTFODOCK(std::string teamAndPosName,
                                 Drivebase *drivebase);

  static frc2::Command *moveToDefense(std::string teamAndPosName,
                                      Drivebase *drivebase);

  static frc2::Command *JustCharge(std::string teamAndPosName,
                                   Drivebase *drivebase);

  static frc2::Command *ScoreThenCharge(std::string teamAndPosName,
                                        Drivebase *drivebase,
                                        IntakeDeployment *intakeDeployment,
                                        IntakeClamp *intakeClamp);

  static frc2::Command *ScoreThenEndNearGamePieceCommand(
      std::string teamAndPosName, Drivebase *drivebase,
      IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp);
  static frc2::Command *DropGamePieceThenGTFOCommand(
      std::string teamAndPosName, Drivebase *drivebase,
      IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp);

 private:
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Drivebase m_drivebase;
  IntakeClamp m_intakeClamp;
  IntakeDeployment m_intakeDeployment;

  frc::SendableChooser<frc2::Command *> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command *> m_RobotSequenceAutonomousOptions;

  void ConfigureBindings();
  void AddTestButtonsToSmartDashboard();

  bool isInverted = true;
  frc::SlewRateLimiter<units::scalar> m_leftSpeedLimiter{
      OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT};
  frc::SlewRateLimiter<units::scalar> m_rightSpeedLimiter{
      OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT};
};
