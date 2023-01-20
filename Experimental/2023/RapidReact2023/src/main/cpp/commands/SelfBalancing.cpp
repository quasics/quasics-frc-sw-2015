// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"

SelfBalancing::SelfBalancing(Drivebase* drivebase) {
  AddRequirements(drivebase);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  pid.Reset();
}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  //Leave Space for the feed forward thing
  units::degree_t currentAngle = m_drivebase->GetAngle();
  //double power = pid.Calculate(currentAngle.convert<double>(), 0.0);
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool SelfBalancing::IsFinished() {
  return false;
}
