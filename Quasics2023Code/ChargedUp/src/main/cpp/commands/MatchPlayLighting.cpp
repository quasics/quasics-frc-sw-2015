// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MatchPlayLighting.h"

#include <frc/DriverStation.h>

MatchPlayLighting::MatchPlayLighting(Lighting* lighting) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void MatchPlayLighting::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void MatchPlayLighting::Execute() {
}

// Called once the command ends or is interrupted.
void MatchPlayLighting::End(bool interrupted) {
}

// Returns true when the command should end.
bool MatchPlayLighting::IsFinished() {
  return false;
}
