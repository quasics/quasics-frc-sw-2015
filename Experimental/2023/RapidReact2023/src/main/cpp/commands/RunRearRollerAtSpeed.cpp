// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunRearRollerAtSpeed.h"

RunRearRollerAtSpeed::RunRearRollerAtSpeed(Shooter* shooter, double speed)
    : m_shooter(shooter), m_rearRollerSpeed(speed) {
  AddRequirements(m_shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunRearRollerAtSpeed::Initialize() {
  m_shooter->SetRollerSpeed(m_rearRollerSpeed);
}

// Called once the command ends or is interrupted.
void RunRearRollerAtSpeed::End(bool interrupted) {
  m_shooter->Stop();
}
