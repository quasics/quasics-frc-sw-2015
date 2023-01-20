// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * Command to make the robot's lights "breath" (cycle between dark and light)
 * with a given color (expressed as an RGB triplet) and maximum intensity
 * (brightness).
 */
class BreathingLights
    : public frc2::CommandHelper<frc2::CommandBase, BreathingLights> {
 public:
  /**
   * Constructor.
   *
   * BUG(Matthew): Why are you providing a maximum intensity here?  Isn't this
   * redundant (and potentially in conflict) with expressing the intensity in
   * terms of the RGB value?  For example, say it's color of (128, 0, 0) and
   * intensity is .5: right now, that means that the maximum light intensity
   * will wind up being RGB (64, 0, 0).  But if that's intended, then why not
   * just set *that* as the color?
   *
   * @param lights  lighting subsystem
   * @param r       red component for the lights, as a value from 0..255
   * @param g       green component for the lights, as a value from 0..255
   * @param b       blue component for the lights, as a value from 0..255
   */
  BreathingLights(Lighting* lights, int r, int g, int b, double intensity);

  // Standard command functions.
 public:
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  // Data members.
 private:
  Lighting* m_lighting;

  // Color values for the LED strip.
  const int red;
  const int green;
  const int blue;

  const double intensityPercent;
  double currentIntensityPercent = 0;

  // (Matthew): Why do you need both of these?  If the increment is
  // positive (or negative),then you know your breathing in (or out).
  // Alternatively, if you know you're breathing in (or out) then you
  // know that you're incrementing (or decrementing) the current
  // intensity.(done)
  double increment = 0.01;
};
