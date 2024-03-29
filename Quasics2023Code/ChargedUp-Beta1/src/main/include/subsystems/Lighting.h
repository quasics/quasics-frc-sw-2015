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

  /**
   * Sets the entire strip to the color specified by a LEDData value.
   */
  void SetAllToColor(const frc::AddressableLED::LEDData& color);

  void SetLightColors(
      std::function<frc::AddressableLED::LEDData(int)> colorFunction);

  /** Convenience function to return the # of LEDs on the robot. */
  static int GetNumberOfLEDs() {
    return LightingValues::PIXEL_NUMBER;
  }

  /**
   * Convenience function to decide if given LED position is on the front of
   * the robot.
   *
   * @param position  position of an LED on the strip (from 0 to length-1)
   * @return true if the LED is on the front of the robot
   */
  static bool IsFrontSideLED(int position) {
    return position >= (GetNumberOfLEDs() / 2);
  }

  /**
   * Convenience function to decide if given LED position is on the rear of
   * the robot.
   *
   * @param position  position of an LED on the strip (from 0 to length-1)
   * @return true if the LED is on the rear of the robot
   */
  static bool IsRearSideLED(int position) {
    return position < (GetNumberOfLEDs() / 2);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetDefaultLighting();

  // Useful color constants.
 public:
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData BLACK;
  static const frc::AddressableLED::LEDData GREEN;
  static const frc::AddressableLED::LEDData BLUE;
  static const frc::AddressableLED::LEDData CYAN;  // Light blue
  static const frc::AddressableLED::LEDData RED;
  static const frc::AddressableLED::LEDData PINK;
  static const frc::AddressableLED::LEDData ORANGE;
  static const frc::AddressableLED::LEDData PURPLE;

 private:
  /** The underlying object that provides actual control of the strip. */
  frc::AddressableLED m_led{LightingValues::PORT_NUMBER};

  /** The buffer used to send individual LED colors to the strip. */
  std::array<frc::AddressableLED::LEDData, LightingValues::PIXEL_NUMBER>
      m_ledBuffer;
};
