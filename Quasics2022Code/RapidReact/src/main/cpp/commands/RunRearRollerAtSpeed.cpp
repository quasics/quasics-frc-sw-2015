// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunRearRollerAtSpeed.h"

RunRearRollerAtSpeed::RunRearRollerAtSpeed(RearRoller* rearRoller, double speed)
    : m_rearRoller(rearRoller), m_rearRollerSpeed(speed) {
  AddRequirements(m_rearRoller);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunRearRollerAtSpeed::Initialize() {
  m_rearRoller->SetRollerSpeed(m_rearRollerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunRearRollerAtSpeed::Execute() {
  m_rearRoller->SetRollerSpeed(m_rearRollerSpeed);
}

// Called once the command ends or is interrupted.
void RunRearRollerAtSpeed::End(bool interrupted) {
  m_rearRoller->Stop();
}

// Returns true when the command should end.
bool RunRearRollerAtSpeed::IsFinished() {
  return false;
}
