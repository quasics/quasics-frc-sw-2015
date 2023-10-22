// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example to control the lights on the robot, showing a "moving rainbow"
 * effect.
 */
class RainbowLighting
    : public frc2::CommandHelper<frc2::Command, RainbowLighting> {
 public:
  /**
   * Constructor.
   *
   * @param lighting             the lighting subsystem
   * @param delayBeforeAdvancing how long before the LEDs' colors advance to the
   *                             next value
   * @param extraGapBetweenColors any additional distance on the color wheel
   *                              between adjacent pixels (e.g., allowing a
   *                              shorter strip to show more pronounced
   *                              variation)
   */
  RainbowLighting(Lighting* lighting,
                  units::second_t delayBeforeAdvancing = 0_s,
                  int extraGapBetweenColors = 0);

  // Standard functions for a Command.
 public:
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  // Data members.
 private:
  Lighting* const m_lighting;
  units::second_t m_delayBeforeAdvancing;
  int m_extraGapBetweenColors;
  std::function<frc::AddressableLED::LEDData(int pos)> m_colorFunction;
  frc::Timer m_timer;
  int m_offset = 0;
};
