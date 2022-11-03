// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

class Lighting : public frc2::SubsystemBase {
 public:
  static const frc::AddressableLED::LEDData BLACK;
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData RED;
  static const frc::AddressableLED::LEDData GREEN;
  static const frc::AddressableLED::LEDData BLUE;

  typedef std::function<frc::AddressableLED::LEDData(int pos)> ColorFunction;

 public:
  Lighting();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetStripColor(int red, int green, int blue);
  void SetStripColor(frc::AddressableLED::LEDData color);
  void SetStripColors(ColorFunction colorFunction);

 private:
  frc::AddressableLED m_ledStrip{LightingValues::PWM_PORT};
  std::array<frc::AddressableLED::LEDData, LightingValues::NUM_LIGHTS>
      m_ledBuffer;
};
