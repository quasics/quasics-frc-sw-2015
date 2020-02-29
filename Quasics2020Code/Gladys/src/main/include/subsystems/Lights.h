/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>

class Lights : public frc2::SubsystemBase {
 public:
  Lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  /**
   * Sets a single color across the whole (logical) strip.
   * @param red    the red component for the color (0-255)
   * @param green  the green component for the color (0-255)
   * @param blue   the blue component for the color (0-255)
   */
  void SetLightsColor(int red, int green, int blue);

  void TurnOff();

 private:
   static constexpr int kLength = 60;
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
};
