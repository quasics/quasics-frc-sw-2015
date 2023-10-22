// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/clamp/ClampWithIntakeAtSpeedForTime.h"

ClampWithIntakeAtSpeedForTime::ClampWithIntakeAtSpeedForTime(
    IntakeClamp* clamp, double power, units::second_t time)
    : m_IntakeClamp(clamp), m_clampPower(std::abs(power)), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_IntakeClamp);
  SetName("ClampWithIntakeAtSpeedForTime");
}

// Called when the command is initially scheduled.
void ClampWithIntakeAtSpeedForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_IntakeClamp->SetIntakeClampSpeed(m_clampPower);
}

// Called repeatedly when this Command is scheduled to run
void ClampWithIntakeAtSpeedForTime::Execute() {
  m_IntakeClamp->SetIntakeClampSpeed(m_clampPower);
}

// Called once the command ends or is interrupted.
void ClampWithIntakeAtSpeedForTime::End(bool interrupted) {
  m_IntakeClamp->Stop();
}

// Returns true when the command should end.
bool ClampWithIntakeAtSpeedForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
