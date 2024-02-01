// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>
// #include <frcUserProgram/util/Color.h>

#include "Constants.h"

// Will need to be updated based on PWM number of lighting & way lighting is
// wired.

class Lighting : public frc2::SubsystemBase {
 public:
  Lighting();

  /**
   * @param r red component, in the range 0..255
   * @param g green component, in the range 0..255
   * @param b blue component, in the range 0..255
   */
  /* void setAllToColor(int r, int g, int b);

  void setAllToColor(const frc::AddressableLED::LedData(int)> colorFunction);

  void setLightColors(
      std::function<frc::AddressableLED::LedData(int)> colorFunction);
  * /

      /* static int GetNumberOfLEDs() {
        return LightingValues::PIXEL_NUMBER;
      } */
  // uncomment when data of lighting strip is present

  /**
   * @param position position of an LED on the strip from 0 to length)
   */

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // void SetDefaultLighting();

 public:
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData BLACK;
  static const frc::AddressableLED::LEDData GREEN;
  static const frc::AddressableLED::LEDData BLUE;
  static const frc::AddressableLED::LEDData RED;
  // static const frc::AddressableLED::LEDData ORANGE;
  // static const frc::AddressableLED::LEDData PURPLE;

 private:
  /* frc::AddressableLED m_led{LightingValues::PORT_NUMBER};

  std::array<frc::AddressableLED::LEDData, LightingValues::PIXEL_NUMBER>
      m_ledBuffer;
      */
};
