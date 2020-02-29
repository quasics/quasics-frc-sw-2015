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

class LightingSubsystem : public frc2::SubsystemBase {
 public:
  LightingSubsystem();

  enum class Color { red, blue, green, black, white };

  void SetStripToSingleColor(Color c);
  void SetStripToSingleRGBColor(uint8_t red, uint8_t green, uint8_t blue);
  void SetStripToSingleHSVColor(uint8_t hue, uint8_t saturation, uint8_t value);
  void TurnOff() {
    SetStripToSingleColor(Color::black);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr int kStripLength = 16;
  frc::AddressableLED ledController;
  std::array<frc::AddressableLED::LEDData, kStripLength> lightingBuffer;
};
