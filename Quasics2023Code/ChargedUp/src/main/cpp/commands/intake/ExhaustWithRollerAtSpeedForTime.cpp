// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/ExhaustWithRollerAtSpeedForTime.h"

ExhaustWithRollerAtSpeedForTime::ExhaustWithRollerAtSpeedForTime(
    IntakeRoller* IntakeRoller, double power, units::second_t time)
    : m_intakeRoller(IntakeRoller), m_power(-std::abs(power)), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeRoller);
  SetName("ExhaustWithRollerAtSpeedForTime");
}

// Called when the command is initially scheduled.
void ExhaustWithRollerAtSpeedForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_intakeRoller->SetRollerSpeed(m_power);
}

// Called repeatedly when this Command is scheduled to run
void ExhaustWithRollerAtSpeedForTime::Execute() {
  m_intakeRoller->SetRollerSpeed(m_power);
}

// Called once the command ends or is interrupted.
void ExhaustWithRollerAtSpeedForTime::End(bool interrupted) {
  m_intakeRoller->Stop();
}

// Returns true when the command should end.
bool ExhaustWithRollerAtSpeedForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
