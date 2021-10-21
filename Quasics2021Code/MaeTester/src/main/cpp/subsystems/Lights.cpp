// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"

Lights::Lights() {
  SetSubsystem("Lights");

  // We need to tell the addressable LED thing how many lights it is
  // theoretically equipped with.  (If we go over, no biggie.  But it
  // needs to get a valid length for comparison to the size of the
  // buffer when "SetData()" is called later: if the buffer is too big,
  // that will fail.)
  m_led.SetLength(kLength);

  // Start with all of the lights off.
  TurnStripOff();
}

// This method will be called once per scheduler run
void Lights::Periodic() {}

void Lights::SetStripColor(int red, int green, int blue){
  std::cout << "Setting strip to solid color" << std::endl;
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(red, green, blue);
  }
  m_led.SetData(m_ledBuffer);
}

void Lights::SetStripColor(
    std::function<frc::AddressableLED::LEDData(int position)> colorFcn) {
  std::cout << "Setting strip colors" << std::endl;
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i] = colorFcn(i);
  }
  m_led.SetData(m_ledBuffer);
}

void Lights::TurnStripOff() {
  SetStripColor(0, 0, 0);
}