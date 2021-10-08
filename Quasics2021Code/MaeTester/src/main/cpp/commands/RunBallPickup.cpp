// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunBallPickup.h"

RunBallPickup::RunBallPickup(Intake* intake, double power,
                             units::second_t duration)
    : m_intake(intake), m_power(power), m_duration(duration) {
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void RunBallPickup::Initialize() {
  m_intake->SetBallPickupSpeed(m_power);
  m_timer.Reset();
  m_timer.Start();
}

// Called once the command ends or is interrupted.
void RunBallPickup::End(bool interrupted) {
  m_intake->SetBallPickupSpeed(0);
}

// Returns true when the command should end.
bool RunBallPickup::IsFinished() {
  return m_timer.HasElapsed(m_duration);
}
