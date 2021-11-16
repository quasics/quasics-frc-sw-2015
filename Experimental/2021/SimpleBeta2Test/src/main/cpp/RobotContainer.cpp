// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>

#include "commands/RunMotorBriefly.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  frc::SmartDashboard::PutData("Run motor forward 100%",
                               new RunMotorBriefly(&m_subsystem, 1.0));
  frc::SmartDashboard::PutData("Run motor backward 30%",
                               new RunMotorBriefly(&m_subsystem, -0.30));
  frc::SmartDashboard::PutData(
      "Say something", new frc2::PrintCommand("Please don't push me again."));
  /* new frc2::InstantCommand(
          [this]() {
    shooter.SetServoPosition(0.0); }, {&shooter}));
    */
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
