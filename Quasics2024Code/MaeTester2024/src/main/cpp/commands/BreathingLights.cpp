// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BreathingLights.h"

// Just going to adjust green for now.  Affect others later.
BreathingLights::BreathingLights(Lights*lights)
  : m_lights(lights),
  red(0),
  green(0),
  blue(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lights);
}

// Called when the command is initially scheduled.
void BreathingLights::Initialize() {
  m_lights->SetStripColor(0, 0, 0);
  red = 0;
  green = 0;
  blue = 0;
  isIncrementing = true;
}

// Called repeatedly when this Command is scheduled to run
void BreathingLights::Execute() {
  if (green >= 255) {
    isIncrementing = false;
  }
  if (green <= 0) {
    isIncrementing = true;
  }
  if (isIncrementing && green < 255) {
    green = green + 1;
  }
  if (!isIncrementing && green > 0) {
    green = green - 1;
  }
  m_lights->SetStripColor(0, green, 0);
}

// Called once the command ends or is interrupted.
void BreathingLights::End(bool interrupted) {
  m_lights->SetStripColor(0, 0, 0);
}

// Returns true when the command should end.
bool BreathingLights::IsFinished() {
  return false;
}
