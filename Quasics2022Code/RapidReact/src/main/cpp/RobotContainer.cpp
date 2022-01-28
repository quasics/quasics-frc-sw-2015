// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/MoveRobotTestCommand.h"
#include "subsystems\Drivebase.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  AddTestButtonToSmartDasboard();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

void RobotContainer::AddTestButtonToSmartDasboard() {
  frc::SmartDashboard::PutData("Test Button Do Something", new MoveRobotTestCommand(&m_Drivebase, 0.2));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
