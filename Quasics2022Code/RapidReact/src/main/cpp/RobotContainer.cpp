// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/DriveAtPowerForMeters.h"
#include "commands/MoveRobotTestCommand.h"
#include "commands/RunConveyorAtSpeed.h"
#include "commands/RunIntakeAtSpeed.h"
#include "commands/RunShooterAtSpeed.h"
#include "commands/ShootForTime.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  TankDrive tankDrive(&m_drivebase, &m_driverStick);
  m_drivebase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureJoystickButtonBindings();
  AddTestButtonToSmartDasboard();
  AddAutonomousCommandsToSmartDashboard();
}

void RobotContainer::ConfigureJoystickButtonBindings() {
  // Configure your button bindings here
}

// Note: 0.65 seems to be reasonable power for the high goal.
void RobotContainer::AddTestButtonToSmartDasboard() {
  frc::SmartDashboard::PutData("Test Button Do Something",
                               new MoveRobotTestCommand(&m_drivebase, 0.2));
  frc::SmartDashboard::PutData("Run Shooter FlyWheel",
                               new RunShooterAtSpeed(&m_shooter, 0.65));
  frc::SmartDashboard::PutData("Run Intake at 20 percent power forward",
                               new RunIntakeAtSpeed(&m_intake, 0.2));
  frc::SmartDashboard::PutData("Run Intake at 30 percent power backward",
                               new RunIntakeAtSpeed(&m_intake, -0.3));
  frc::SmartDashboard::PutData("Run Conveyor at 20 percent speed forward",
                               new RunConveyorAtSpeed(&m_conveyor, 0.2));
  frc::SmartDashboard::PutData("Run Conveyor at 30 percent speed backward",
                               new RunConveyorAtSpeed(&m_conveyor, -0.3));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard() {
  m_autonomousOptions.SetDefaultOption(
      "Do Nothing", new frc2::PrintCommand("I decline to do anything."));
  m_autonomousOptions.AddOption(
      "Move Forward 1m at 50 percent power",
      new DriveAtPowerForMeters(&m_drivebase, 0.5, 1));
  m_autonomousOptions.AddOption(
      "Shoot at 20 percent power for 3 seconds",
      new ShootForTime(&m_shooter, -0.2, units::second_t(3)));
  m_autonomousOptions.AddOption(
      "Shoot 0.2 for 2, move 0.2 for 1",
      ShootAndMoveCommand(-0.2, units::second_t(2), 0.2, 1));
  frc::SmartDashboard::PutData("Auto mode", &m_autonomousOptions);
}

frc2::SequentialCommandGroup* RobotContainer::ShootAndMoveCommand(
    double powerShoot, units::second_t timeShoot, double powerMove,
    double distanceToMove) {
  // Holds the sequence of the commands to be executed as a group.
  std::vector<std::unique_ptr<frc2::Command>> commands;

  // Add each of the individual commands to the sequence.
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new ShootForTime(&m_shooter, powerShoot, timeShoot))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(&m_drivebase, powerMove, distanceToMove))));

  // Builds the command group object.
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_autonomousOptions.GetSelected();
}
