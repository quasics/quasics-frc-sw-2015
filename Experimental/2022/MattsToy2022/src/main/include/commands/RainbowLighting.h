// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example of lighting control on the robot, which will cycle the lights
 * through a "rainbow" sequence.
 */
class RainbowLighting
    : public frc2::CommandHelper<frc2::CommandBase, RainbowLighting> {
 public:
  RainbowLighting(Lighting* lighting,
                  units::second_t secondsBeforeAdvancing = 0_s,
                  int extraGapBetweenColors = 0);

  void Initialize() override;

  void Execute() override;

 private:
  void UpdateStrip();

 private:
  /** Subsystem with which we're interacting. */
  Lighting* m_lighting;

  /** Maximum legal value in WPILib for hues. */
  static constexpr int MAX_HUE = 180;

  /**
   * Used to store an "offset" used to translate a physical LED's position to an
   * effective position around the range of hues. This is reset in initialize(),
   * and then updated every time that execute() is called, in order to advance
   * the rainbow a step.
   */
  int m_offset = 0;

  const units::second_t m_secondsBeforeAdvancing;

  int m_extraGapBetweenColors;

  /** Used to control delay in advancing colors. */
  frc::Timer m_timer;
};
