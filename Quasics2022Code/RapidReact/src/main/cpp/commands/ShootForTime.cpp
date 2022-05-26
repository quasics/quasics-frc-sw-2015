// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootForTime.h"

ShootForTime::ShootForTime(Shooter* shooter, double power, units::second_t time,
                           double rollerSpeed)
    : m_shooter(shooter),
      m_power(power),
      m_time(time),
      m_rollerSpeed(rollerSpeed) {
  AddRequirements(m_shooter);
}

// Called when the command is initially scheduled.
void ShootForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_shooter->SetFlywheelSpeed(m_power);
  m_shooter->SetRollerSpeed(m_rollerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ShootForTime::Execute() {
  m_shooter->SetFlywheelSpeed(m_power);
  m_shooter->SetRollerSpeed(m_rollerSpeed);
}

// Called once the command ends or is interrupted.
void ShootForTime::End(bool interrupted) {
  m_shooter->Stop();
}

// Returns true when the command should end.
bool ShootForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
