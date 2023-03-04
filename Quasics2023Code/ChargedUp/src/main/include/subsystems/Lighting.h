// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Lighting : public frc2::SubsystemBase {
 public:
  Lighting();

  /**
   * Sets the entire strip to the color specified by an RGB "triplet".
   *
   * @param r  red component, in the range 0..255 (off to full intensity)
   * @param g  green component, in the range 0..255 (off to full intensity)
   * @param b  blue component, in the range 0..255 (off to full intensity)
   */
  void SetAllToColor(int r, int g, int b);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::AddressableLED m_led{LightingValues::PORT_NUMBER};
  std::array<frc::AddressableLED::LEDData, LightingValues::PIXEL_NUMBER>
      m_ledBuffer;
};
