// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/lighting/SetLightsToColor.h"

#include <iostream>

SetLightsToColor::SetLightsToColor(Lighting* lighting, int r, int g, int b) {
  m_r = r;
  m_g = g;
  m_b = b;
  m_lighting = lighting;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
  SetName("SetLightsToColor");
}

// Called repeatedly when this Command is scheduled to run
void SetLightsToColor::Execute() {
  m_lighting->SetAllToColor(m_r, m_g, m_b);
}

bool SetLightsToColor::IsFinished() {
  return false;
}
