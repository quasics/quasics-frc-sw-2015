// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

Lighting::Lighting() {
  m_led.SetLength(LightingValues::PIXEL_NUMBER);
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; i++) {
    m_ledBuffer[i].SetRGB(0, 255, 0);
  }
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
