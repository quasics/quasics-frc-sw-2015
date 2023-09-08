// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

class Lights : public frc2::SubsystemBase {
 public:
  Lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Sets the strip to a single, solid color, specified as an RGB value.
   * (Each component must be in the range [0..255].)
   */
  void SetStripColor(int red, int green, int blue);

  /**
   * Sets the colors for each pixel on the strip, using the specified helper
   * function to get the color at each position.
   */
  void SetStripColor(
      std::function<frc::AddressableLED::LEDData(int position)> colorFcn);

  /** Turns all of the pixels on the strip off. */
  void TurnStripOff();

 private:
  static constexpr int kLength = 45;
  frc::AddressableLED m_led{PwmIds::LedControl};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
};
