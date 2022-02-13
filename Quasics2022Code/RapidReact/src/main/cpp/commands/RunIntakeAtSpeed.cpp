// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunIntakeAtSpeed.h"

RunIntakeAtSpeed::RunIntakeAtSpeed(Intake* intake, double speed)
    : m_intake(intake), m_intakeSpeed(speed) {
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void RunIntakeAtSpeed::Initialize() {
  m_intake->SetIntakeSpeed(m_intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunIntakeAtSpeed::Execute() {
  m_intake->SetIntakeSpeed(m_intakeSpeed);
}

// Called once the command ends or is interrupted.
void RunIntakeAtSpeed::End(bool interrupted) {
  m_intake->Stop();
}

// Returns true when the command should end.
bool RunIntakeAtSpeed::IsFinished() {
  return false;
}
