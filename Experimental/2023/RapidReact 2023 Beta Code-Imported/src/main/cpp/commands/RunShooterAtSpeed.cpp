// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunShooterAtSpeed.h"

RunShooterAtSpeed::RunShooterAtSpeed(Shooter* shooter, double flyWheelSpeed,
                                     double rollerSpeed)
    : m_shooter(shooter),
      m_flyWheelSpeed(flyWheelSpeed),
      m_rollerSpeed(rollerSpeed) {
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void RunShooterAtSpeed::Initialize() {
  m_shooter->SetFlywheelSpeed(m_flyWheelSpeed);
  m_shooter->SetRollerSpeed(m_rollerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunShooterAtSpeed::Execute() {
  m_shooter->SetFlywheelSpeed(m_flyWheelSpeed);
  m_shooter->SetRollerSpeed(m_rollerSpeed);
}

// Called once the command ends or is interrupted.
void RunShooterAtSpeed::End(bool interrupted) {
  m_shooter->Stop();
}

// Returns true when the command should end.
bool RunShooterAtSpeed::IsFinished() {
  return false;
}
