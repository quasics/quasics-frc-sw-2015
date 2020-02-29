/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/LightingSubsystem.h"

#include "Constants.h"

LightingSubsystem::LightingSubsystem()
    : ledController(PwmAssignments::LedStripId) {
  ledController.SetLength(kStripLength);

  // Initial color is "off".
  SetStripToSingleRGBColor(0, 0, 0);

  ledController.Start();
}

// This method will be called once per scheduler run
void LightingSubsystem::Periodic() {
}

void LightingSubsystem::SetStripToSingleRGBColor(uint8_t red, uint8_t green,
                                                 uint8_t blue) {
  for (auto& cell : lightingBuffer) {
    cell.SetRGB(red, green, blue);
  }
  ledController.SetData(lightingBuffer);
}

void LightingSubsystem::SetStripToSingleHSVColor(uint8_t hue,
                                                 uint8_t saturation,
                                                 uint8_t value) {
  for (auto& cell : lightingBuffer) {
    cell.SetHSV(hue, saturation, value);
  }
  ledController.SetData(lightingBuffer);
}

void LightingSubsystem::SetStripToSingleColor(Color c) {
  switch (c) {
    case Color::black:
      SetStripToSingleRGBColor(0, 0, 0);
      break;
    case Color::white:
      SetStripToSingleRGBColor(255, 255, 255);
      break;
    case Color::red:
      SetStripToSingleRGBColor(255, 0, 0);
      break;
    case Color::blue:
      SetStripToSingleRGBColor(0, 0, 255);
      break;
    case Color::green:
      SetStripToSingleRGBColor(0, 255, 0);
      break;
  }
}