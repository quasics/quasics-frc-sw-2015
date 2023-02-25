// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ReleaseWithIntake.h"

ReleaseWithIntake::ReleaseWithIntake(IntakeClamp* clamp, double power)
    : m_IntakeClamp(clamp), m_clampPower(-std::abs(power)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_IntakeClamp);
}

// Called when the command is initially scheduled.
void ReleaseWithIntake::Initialize() {
  m_IntakeClamp->EnableBraking(true);
  m_IntakeClamp->SetIntakeClampSpeed(m_clampPower);
}

// Called repeatedly when this Command is scheduled to run
void ReleaseWithIntake::Execute() {
  m_IntakeClamp->SetIntakeClampSpeed(m_clampPower);
}

// Called once the command ends or is interrupted.
void ReleaseWithIntake::End(bool interrupted) {
  m_IntakeClamp->Stop();
  m_IntakeClamp->EnableBraking(true);
}

// Returns true when the command should end.
bool ReleaseWithIntake::IsFinished() {
  return false;
}
