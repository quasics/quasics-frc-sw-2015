// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivebase.h"

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

  frc2::Command* GetAutonomousTeamAndStationCommand();
  frc2::Command* GetAutonomousRobotSequenceCommand();

  double GetDriveSpeedScalingFactor();

  void AddTeamAndStationSelectorToSmartDashboard();
  void AddRobotSequenceSelectorToSmartDashboard();

  frc2::SequentialCommandGroup* RedAndBlueDriveStation2GTFO();

 private:
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};
  

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Drivebase m_drivebase;

  frc::SendableChooser<frc2::Command*> m_TeamAndStationAutonomousOptions;
  frc::SendableChooser<frc2::Command*> m_RobotSequenceAutonomousOptions;

  void ConfigureBindings();
  void AddTestButtonsToSmartDashboard();

  bool isInverted = true;
  frc::SlewRateLimiter<units::scalar> m_leftSpeedLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSpeedLimiter;
};
