// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example command to control the lights on the robot, using one color for
 * the front LEDs and a different color for the rear ones.
 */
class SplitLightingExample
    : public frc2::CommandHelper<frc2::CommandBase, SplitLightingExample> {
 public:
  /** Constructor. */
  SplitLightingExample(Lighting* lighting,
                       frc::AddressableLED::LEDData frontColor,
                       frc::AddressableLED::LEDData rearColor);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  // The function that will yield the color for each LED on the strips.
  frc::AddressableLED::LEDData ColorFunction(int pos);

  // Helper function, invoked by Initialize() and Execute().
  void UpdateColors();

 private:
  Lighting* const m_lighting;
  const frc::AddressableLED::LEDData m_frontColor;
  const frc::AddressableLED::LEDData m_rearColor;
};
