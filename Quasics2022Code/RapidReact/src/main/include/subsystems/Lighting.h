// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

/**
 * Lighting control subsystem, handling an addressable LED strip.
 */
class Lighting : public frc2::SubsystemBase {
 public:
  /**
   * Convenience type, providing shortcut values for some "well-defined"
   * colors.
   */
  enum class StockColor { Red, Green, Blue, White, Black };

  /** Constructor. */
  Lighting();

  /** Sets the entire strip to the specified (stock) color. */
  void SetAllToColor(StockColor c);

  /**
   * Sets the entire strip to the color specified by an RGB "triplet".
   *
   * @param r  red component, in the range 0..255 (off to full intensity)
   * @param g  green component, in the range 0..255 (off to full intensity)
   * @param b  blue component, in the range 0..255 (off to full intensity)
   */
  void SetAllToColor(int r, int g, int b);

  /**
   * Sets the entire strip to the specified color.
   *
   * Note: the WPILib's idea of RGB triplet values is r/g/b, each as a *double*
   * in the range 0.0 to 1.0.
   *
   * @param c  the color to which the lights should be set
   */
  void SetAllToColor(frc::Color c);

  /**
   * Helper function to convert a StockColor to the corresponding frc::Color.
   */
  static frc::Color Translate(StockColor c);

  // Standard subsystem methods.
 public:
  void Periodic() override;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 private:
  frc::AddressableLED m_ledStrip{LightingValues::PWM_PORT};
  std::array<frc::AddressableLED::LEDData, LightingValues::NUM_LIGHTS>
      m_ledBuffer;
};
