// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunShooter.h"

RunShooter::RunShooter(Shooter &shooter, double shooterSpeed, bool shooting)
    : m_shooter(shooter),
      m_shooterSpeed(shooting ? std::abs(shooterSpeed)
                              : -std::abs(shooterSpeed)),
      m_shooting(shooting) {
  AddRequirements(&m_shooter);
}

// Called when the command is initially scheduled.
void RunShooter::Initialize() {
  m_shooter.SetFlywheelSpeed(m_shooterSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunShooter::Execute() {
  m_shooter.SetFlywheelSpeed(m_shooterSpeed);
}

// Called once the command ends or is interrupted.
void RunShooter::End(bool interrupted) {
  m_shooter.Stop();
}

// Returns true when the command should end.
bool RunShooter::IsFinished() {
  return false;
}
