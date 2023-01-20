// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingLights.h"

BreathingLights::BreathingLights(Lighting* lights, int r, int g, int b,
                                 double intensity)
    : m_lighting(lights),
      red(r),
      green(g),
      blue(b),
      intensityPercent(intensity) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void BreathingLights::Initialize() {
  m_lighting->SetAllToColor(red * currentIntensityPercent,
                            green * currentIntensityPercent,
                            blue * currentIntensityPercent);
}

// Called repeatedly when this Command is scheduled to run
void BreathingLights::Execute() {
  if (currentIntensityPercent >= intensityPercent) {
    increment = -0.01;
  }
  if (currentIntensityPercent <= 0) {
    increment = 0.01;
  }
  m_lighting->SetAllToColor(red * currentIntensityPercent,
                            green * currentIntensityPercent,
                            blue * currentIntensityPercent);
  currentIntensityPercent += increment;
}

// Called once the command ends or is interrupted.
void BreathingLights::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool BreathingLights::IsFinished() {
  return false;
}
