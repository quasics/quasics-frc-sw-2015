// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <frc/AddressableLED.h>

class Lights : public frc2::SubsystemBase {
 public:
  Lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetStripColor(int red, int green, int blue);
  void TurnStripOff();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr int kLength = 60;
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
};
