// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

/**
 * Subsystem providing access to the LED strip lighting on the robot.
 *
 * Assumptions:
 *   * There are 2*N LEDs on the (logical) LED strip.  (Actually 2 strips wired
 *     end-to-end.)
 *   * The first N LEDs are on the back of the robot.
 *   * The other N LEDs are on the front of the robot.
 */
class Lighting : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  Lighting();

  /**
   * Sets the entire strip to the color specified by an RGB "triplet".
   *
   * @param r  red component, in the range 0..255 (off to full intensity)
   * @param g  green component, in the range 0..255 (off to full intensity)
   * @param b  blue component, in the range 0..255 (off to full intensity)
   */
  void SetAllToColor(int r, int g, int b);

  void SetLightColors(
      std::function<frc::AddressableLED::LEDData(int)> colorFunction);

  /** Convenience function to return the # of LEDs on the robot. */
  int GetNumberOfLEDs() {
    return LightingValues::PIXEL_NUMBER;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Useful color constants.
 public:
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData BLACK;

 private:
  /** The underlying object that provides actual control of the strip. */
  frc::AddressableLED m_led{LightingValues::PORT_NUMBER};

  /** The buffer used to send individual LED colors to the strip. */
  std::array<frc::AddressableLED::LEDData, LightingValues::PIXEL_NUMBER>
      m_ledBuffer;
};
