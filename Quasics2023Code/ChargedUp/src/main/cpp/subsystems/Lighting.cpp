// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

const frc::AddressableLED::LEDData Lighting::WHITE{255, 255, 255};
const frc::AddressableLED::LEDData Lighting::BLACK{0, 0, 0};

Lighting::Lighting() {
  m_led.SetLength(LightingValues::PIXEL_NUMBER);
  SetAllToColor(0, 255, 0);
  m_led.Start();
}

void Lighting::SetAllToColor(int r, int g, int b) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; i++) {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
  m_led.SetData(m_ledBuffer);
}

void Lighting::SetLightColors(
    std::function<frc::AddressableLED::LEDData(int)> colorFunction) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; ++i) {
    m_ledBuffer[i] = colorFunction(i);
  }
  m_led.SetData(m_ledBuffer);
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
