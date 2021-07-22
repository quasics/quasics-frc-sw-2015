// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"

Lights::Lights() = default;

// This method will be called once per scheduler run
void Lights::Periodic() {}

void Lights::SetStripColor(int red, int green, int blue){
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(red, green, blue);
  }
  m_led.SetData(m_ledBuffer);
}

void Lights::TurnStripOff(){
  SetStripColor(0, 0, 0);
}