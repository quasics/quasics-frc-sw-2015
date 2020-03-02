/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

/**
 * Sample implementation of a subsystem providing simple control of an
 * addressable LED strip (e.g., NeoPixels, etc.).
 *
 * In previous years, this would have required a secondary processor (e.g.,
 * Arduino) on the robot; as of 2020, this functionality is now available on the
 * Rio.
 */
class LightingSubsystem : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  LightingSubsystem();

  /** Defines a set simple colors that can be requested without knowing RGB
   * triplets. */
  enum class Color { red, blue, green, black, white };

  /**
   * Sets the lights to the requested color.
   *
   * @param c the color to be shown
   */
  void SetStripToSingleColor(Color c);

  /**
   * Sets the lights to the requested color.
   *
   * @param red the R component of an RGB triplet
   * @param green the G component of an RGB triplet
   * @param blue the B component of an RGB triplet
   */
  void SetStripToSingleRGBColor(uint8_t red, uint8_t green, uint8_t blue);

  /**
   * Sets the lights to the requested color.
   *
   * @param hue the H component of an HSV triplet
   * @param saturation the S component of an HSV triplet
   * @param value the V component of an HSV triplet
   */
  void SetStripToSingleHSVColor(uint8_t hue, uint8_t saturation, uint8_t value);

  /**
   * Convenience function to turn off the lights (by setting them to "black").
   */
  void TurnOff() {
    SetStripToSingleColor(Color::black);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   *
   * @todo Remove this method: it's unneeded by this subsystem at present.
   */
  void Periodic();

 private:
  static constexpr int kStripLength =
      16;  ///< Length of the lighting strip (in pixels)
  frc::AddressableLED
      ledController;  ///< WPI controller used to manipulate the lights
  std::array<frc::AddressableLED::LEDData, kStripLength>
      lightingBuffer;  ///< Buffer used to send lighting data to the strip.
};
