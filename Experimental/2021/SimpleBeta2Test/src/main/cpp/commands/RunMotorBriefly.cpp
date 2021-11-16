// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunMotorBriefly.h"

RunMotorBriefly::RunMotorBriefly(ExampleSubsystem* subsystem, double percentage)
    : m_subsystem(subsystem), m_percentage(percentage) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void RunMotorBriefly::Initialize() {
  m_subsystem->SetMotorPower(m_percentage);
  m_timer.Start();
  m_timer.Reset();
}

// Called repeatedly when this Command is scheduled to run
void RunMotorBriefly::Execute() {
  m_subsystem->SetMotorPower(m_percentage);
}

// Called once the command ends or is interrupted.
void RunMotorBriefly::End(bool interrupted) {
  m_subsystem->StopMotor();
}

// Returns true when the command should end.
bool RunMotorBriefly::IsFinished() {
  return m_timer.HasElapsed(5_s);
}
