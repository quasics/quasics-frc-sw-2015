// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunIntakeTimed.h"

RunIntakeTimed::RunIntakeTimed(IntakeRoller &intake, double intakeSpeed,
                               units::second_t time, bool takingIn)
    : m_intake(intake),
      m_intakeSpeed(takingIn ? -std::abs(intakeSpeed) : std::abs(intakeSpeed)),
      m_time(time),
      m_takingIn(takingIn) {
  AddRequirements(&m_intake);
}

// Called when the command is initially scheduled.
void RunIntakeTimed::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_intake.SetRollerSpeed(m_intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunIntakeTimed::Execute() {
  m_intake.SetRollerSpeed(m_intakeSpeed);
}

// Called once the command ends or is interrupted.
void RunIntakeTimed::End(bool interrupted) {
  m_intake.Stop();
}

// Returns true when the command should end.
bool RunIntakeTimed::IsFinished() {
  return m_stopWatch.HasElapsed(m_time);
}
