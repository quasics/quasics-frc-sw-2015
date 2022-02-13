// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Conveyor.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // Public methods, used by the Robot class.
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  // "Helper" functions, used internall.
 private:
  frc2::SequentialCommandGroup* ShootAndMoveCommand(double powerShoot,
                                                    units::second_t timeShoot,
                                                    double powerMove,
                                                    double distanceToMove);

  void ConfigureJoystickButtonBindings();
  void AddTestButtonToSmartDashboard();
  void AddAutonomousCommandsToSmartDashboard();

  // The robot's subsystems and commands are defined here.
 private:
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};

  Shooter m_shooter;
  Drivebase m_drivebase;
  Intake m_intake;
  Conveyor m_conveyor;

  frc::SendableChooser<frc2::Command*> m_autonomousOptions;

  frc2::PrintCommand m_autonomousCommand{
      "We need to do something autonomously...."};
};
