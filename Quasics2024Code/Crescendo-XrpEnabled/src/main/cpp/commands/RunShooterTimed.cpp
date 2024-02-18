// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunShooterTimed.h"

RunShooterTimed::RunShooterTimed(Shooter &shooter, double shooterSpeed,
                                 units::second_t time, bool shooting)
    : m_shooter(shooter),
      m_shooterSpeed(shooting ? std::abs(shooterSpeed)
                              : -std::abs(shooterSpeed)),
      m_time(time),
      m_shooting(shooting) {
  AddRequirements(&m_shooter);
}

// Called when the command is initially scheduled.
void RunShooterTimed::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_shooter.SetFlywheelSpeed(m_shooterSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunShooterTimed::Execute() {
  m_shooter.SetFlywheelSpeed(m_shooterSpeed);
}

// Called once the command ends or is interrupted.
void RunShooterTimed::End(bool interrupted) {
  m_shooter.Stop();
}

// Returns true when the command should end.
bool RunShooterTimed::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
