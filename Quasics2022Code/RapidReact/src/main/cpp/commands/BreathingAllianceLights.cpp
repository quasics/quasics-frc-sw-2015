// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingAllianceLights.h"

BreathingAllianceLights::BreathingAllianceLights(Lighting* lights,
                                                 double intensity)
    : m_lighting(lights), intensityPercent(intensity) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void BreathingAllianceLights::Initialize() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
    isRed = true;
  } else {
    isRed = false;
  }
  if (isRed) {
    m_lighting->SetAllToColor(red * currentIntensityPercent, 0, 0);
  } else {
    m_lighting->SetAllToColor(0, 0, blue * currentIntensityPercent);
  }
}

// Called repeatedly when this Command is scheduled to run
void BreathingAllianceLights::Execute() {
  if (currentIntensityPercent >= intensityPercent) {
    breathingIn = false;
    increment = -0.01;
  }
  if (currentIntensityPercent <= 0) {
    breathingIn = true;
    increment = 0.01;
  }

  if (isRed) {
    m_lighting->SetAllToColor(red * currentIntensityPercent, 0, 0);
  } else {
    m_lighting->SetAllToColor(0, 0, blue * currentIntensityPercent);
  }
  currentIntensityPercent += increment;
}

// Called once the command ends or is interrupted.
void BreathingAllianceLights::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool BreathingAllianceLights::IsFinished() {
  return false;
}
