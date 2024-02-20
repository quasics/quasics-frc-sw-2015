// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

// "stock colors"
const frc::AddressableLED::LEDData Lighting::WHITE{255, 255, 255};
const frc::AddressableLED::LEDData Lighting::BLACK{0, 0, 0};
const frc::AddressableLED::LEDData Lighting::GREEN{0, 255, 0};
const frc::AddressableLED::LEDData Lighting::BLUE{0, 0, 255};
const frc::AddressableLED::LEDData Lighting::RED{255, 0, 0};

Lighting::Lighting() {
  SetName("Lighting");

  /* m_led.SetLength(LightingValues::PIXEL_NUMBER);
  setAllToColor(GREEN);
  m_led.Start(); */
}

/* void Lighting::SetDefaultLighting() {
  setAllToColor(GREEN);
} */

// when called, it sets all lights on lighting strip to selected rgb value or
// stock color
/* void::Lighting::setAllToColor(int r, int g, int b) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; i++) {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
  m_led.SetData(m_ledBuffer);
} */

/* void Lighting::SetLightColors(
  std::function<frc::AddressableLED::LEDData(int)> colorFunction) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; ++i) {
    m_ledBuffer[i] = colorFunction(i);
  }
  m_led.SetData(m_ledBuffer);
  } */

int Lighting::GetNumberOfLEDs() {
  return LightingValues::PIXEL_NUMBER;
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}