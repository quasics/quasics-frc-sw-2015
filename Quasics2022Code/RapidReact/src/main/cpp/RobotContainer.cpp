// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/MoveRobotTestCommand.h"
#include "commands/RunShooterAtSpeed.h"
#include "commands/TankDrive.h"
#include "commands/DriveAtPowerForMeters.h"

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

void RobotContainer::AddTestButtonToSmartDasboard() {
  frc::SmartDashboard::PutData("Test Button Do Something",
                               new MoveRobotTestCommand(&m_drivebase, 0.2));
  frc::SmartDashboard::PutData("Run Shooter FlyWheel", new RunShooterAtSpeed(&m_shooter, 0.5));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard(){
  m_autonoumousOptions.AddOption("Move Forward 1m at 50 percent power", new DriveAtPowerForMeters(&m_drivebase, 0.5, 1));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
