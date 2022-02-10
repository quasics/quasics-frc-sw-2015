// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/PrintCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivebase.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

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
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};

  Shooter m_shooter;
  Drivebase m_drivebase;
  Intake m_intake;

  frc2::PrintCommand m_autonomousCommand{
    "We need to do something autonomously...."};

  frc::SendableChooser<frc2::Command*> m_autonoumousOptions;
  frc2::SequentialCommandGroup* ShootAndMoveCommand(double powerShoot, units::second_t timeShoot, double powerMove, double distanceMove);

  void ConfigureJoystickButtonBindings();
  void AddTestButtonToSmartDasboard();
  void AddAutonomousCommandsToSmartDashboard();
};
