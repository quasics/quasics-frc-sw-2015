// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunIntake.h"

RunIntake::RunIntake(IntakeRoller &intake, double intakeSpeed, bool takingIn)
    : m_intake(intake),
      m_intakeSpeed(takingIn ? -std::abs(intakeSpeed) : std::abs(intakeSpeed)),
      m_takingIn(takingIn) {
  AddRequirements(&m_intake);
}

// Called when the command is initially scheduled.
void RunIntake::Initialize() {
  m_intake.SetRollerSpeed(m_intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunIntake::Execute() {
  m_intake.SetRollerSpeed(m_intakeSpeed);
}

// Called once the command ends or is interrupted.
void RunIntake::End(bool interrupted) {
  m_intake.Stop();
}
