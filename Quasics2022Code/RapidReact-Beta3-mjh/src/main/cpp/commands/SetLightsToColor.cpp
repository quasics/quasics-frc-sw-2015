// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLightsToColor.h"

#include <iostream>

SetLightsToColor::SetLightsToColor(Lighting* lighting, Lighting::StockColor c)
    : m_lighting(lighting), m_color(c) {
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void SetLightsToColor::Execute() {
  m_lighting->SetAllToColor(m_color);
}
