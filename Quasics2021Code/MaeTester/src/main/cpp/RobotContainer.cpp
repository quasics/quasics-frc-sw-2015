// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/Drivebase.h"
#include "commands/DoASpin.h"
#include "Constants.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
}

void RobotContainer::ConfigureButtonBindings() {
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

void RobotContainer::ConfigureSmartDashboard() {
  frc::SmartDashboard::PutData("Do those spinnin",
                               new DoASpin(&drivebase));
}
