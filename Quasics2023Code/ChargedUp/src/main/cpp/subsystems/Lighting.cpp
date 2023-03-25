// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

////////////////////////////////////////////////////////////////////////////
// Simple "stock colors"
////////////////////////////////////////////////////////////////////////////

const frc::AddressableLED::LEDData Lighting::WHITE{255, 255, 255};
const frc::AddressableLED::LEDData Lighting::BLACK{0, 0, 0};
const frc::AddressableLED::LEDData Lighting::GREEN{0, 255, 0};
const frc::AddressableLED::LEDData Lighting::BLUE{0, 0, 255};
const frc::AddressableLED::LEDData Lighting::RED{255, 0, 0};

////////////////////////////////////////////////////////////////////////////
// The following are based on data from https://htmlcolorcodes.com/colors/
////////////////////////////////////////////////////////////////////////////

// "Bright orange"
const frc::AddressableLED::LEDData Lighting::ORANGE{255, 172, 28};
// "Bright purple"
const frc::AddressableLED::LEDData Lighting::PURPLE{191, 64, 191};
// "Hot pink"
const frc::AddressableLED::LEDData Lighting::PINK{255, 105, 180};

Lighting::Lighting() {
  SetName("Lighting");

  m_led.SetLength(LightingValues::PIXEL_NUMBER);
  SetAllToColor(GREEN);
  m_led.Start();
}

void Lighting::SetAllToColor(int r, int g, int b) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; i++) {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
  m_led.SetData(m_ledBuffer);
}

void Lighting::SetAllToColor(const frc::AddressableLED::LEDData& color) {
  for (int i = 0; i < LightingValues::PIXEL_NUMBER; i++) {
    m_ledBuffer[i] = color;
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
