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

  typedef std::function<frc::Color(int pos)> FrcColorFunction;

  /**
   * Calls the provided function once for each cell on the strip, and sets
   * the cell's color to the value that's returned.
   *
   * @param colorFunction  a function returning the color to use for the
   *                       specified cell (pixel) on the strip
   */
  void SetEachCellToColor(FrcColorFunction colorFunction);

  /**
   * Helper function to convert a StockColor to the corresponding frc::Color.
   */
  static frc::Color Translate(StockColor c);

  /**
   * Helper function to convert a StockColor to the corresponding color data
   * used for the strip's buffer.
   */
  static frc::AddressableLED::LEDData TranslateToLEDData(StockColor c) {
    return TranslateToLEDData(Translate(c));
  }

  /**
   * Helper function to convert an frc::Color to the corresponding color data
   * used for the strip's buffer.
   */
  static frc::AddressableLED::LEDData TranslateToLEDData(frc::Color c) {
    return {int(c.red * 255), int(c.green * 255), int(c.blue * 255)};
  }

  int GetNumberOfLEDs() {
    return m_ledBuffer.size();
  }

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
