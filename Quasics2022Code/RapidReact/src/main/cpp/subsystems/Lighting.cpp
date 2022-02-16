// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

Lighting::Lighting() {
  m_ledStrip.SetLength(m_ledBuffer.size());
}

void Lighting::SetAllToColor(StockColor c) {
  SetAllToColor(Translate(c));
}

void Lighting::SetAllToColor(int r, int g, int b) {
  SetAllToColor(frc::Color(r / 255.0, g / 255.0, b / 255.0));
}

void Lighting::SetAllToColor(frc::Color c) {
  for (int i = 0; i < int(m_ledBuffer.size()); i++) {
    m_ledBuffer[i].SetLED(c);
  }
  m_ledStrip.SetData(m_ledBuffer);
}

frc::Color Lighting::Translate(StockColor c) {
  if (c == StockColor::Red) {
    return frc::Color(255, 0, 0);
  } else if (c == StockColor::Green) {
    return frc::Color(0, 255, 0);
  } else if (c == StockColor::Blue) {
    return frc::Color(0, 0, 255);
  } else if (c == StockColor::White) {
    return frc::Color(255, 255, 255);
  } else {
    return frc::Color(0, 0, 0);
  }
}
// This method will be called once per scheduler run
void Lighting::Periodic() {
}
