// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

class Lighting : public frc2::SubsystemBase {
 public:
  enum class StockColor { Red, Green, Blue, White, Black };
  Lighting();

  void SetAllToColor(StockColor c);

  void SetAllToColor(int r, int g, int b);

  void SetAllToColor(frc::Color c);

  frc::Color Translate(StockColor c);

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::AddressableLED m_ledStrip{LightingValues::PWM_PORT};
  std::array<frc::AddressableLED::LEDData, LightingValues::NUM_LIGHTS>
      m_ledBuffer;
};
