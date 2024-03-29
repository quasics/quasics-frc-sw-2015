// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/RunMotor.h"
#include "subsystems/MotorOne.h"
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  AddMotorControlToDashboard();

}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}




frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

void RobotContainer::AddMotorControlToDashboard() {
  frc::SmartDashboard::PutData("Run motor at 100% power",
                               new RunMotor(&motorOne, 1));


}

