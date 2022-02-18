// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingAllianceLights.h"

BreathingAllianceLights::BreathingAllianceLights() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BreathingAllianceLights::Initialize() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
    BreathingLights(m_lighting, 255, 0, 0, 1.00);
  } else {
    BreathingLights(m_lighting, 0, 0, 255, 1.00);
  }
}

// Called repeatedly when this Command is scheduled to run
void BreathingAllianceLights::Execute() {
}

// Called once the command ends or is interrupted.
void BreathingAllianceLights::End(bool interrupted) {
}

// Returns true when the command should end.
bool BreathingAllianceLights::IsFinished() {
  return false;
}
