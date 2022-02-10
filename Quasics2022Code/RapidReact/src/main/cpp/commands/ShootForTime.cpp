// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootForTime.h"

ShootForTime::ShootForTime(Shooter* shooter, double power, units::second_t time) : m_shooter(shooter), power(power), time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_shooter);
}

// Called when the command is initially scheduled.
void ShootForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_shooter ->SetFlywheelSpeed(power);
}

// Called repeatedly when this Command is scheduled to run
void ShootForTime::Execute() {
  m_shooter ->SetFlywheelSpeed(power);
}

// Called once the command ends or is interrupted.
void ShootForTime::End(bool interrupted) {
  m_shooter -> Stop();
}

// Returns true when the command should end.
bool ShootForTime::IsFinished() {
  if(m_stopWatch.HasElapsed(time)) {
    return true;
  }
  return false;
}
