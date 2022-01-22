// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <array>

#include "Constants.h"

class Lighting : public frc2::SubsystemBase {
 public:
  Lighting();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::AddressableLED m_ledStrip{LightingValues::PWM_PORT};
  std::array<frc::AddressableLED::LEDData, LightingValues::NUM_LIGHTS>
      m_ledBuffer;
};
