// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BreathingAllianceLights
    : public frc2::CommandHelper<frc2::CommandBase, BreathingAllianceLights> {
 public:
  BreathingAllianceLights(Lighting* lights, double intensity);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Lighting* m_lighting;
  // BUG(Matthew): Why have 2 variables here, especially when they're just
  // placeholders for the same constant (255/full intensity) in each case?
  // Why not just have one constant, and "pop it into" the right point of
  // the triplet, based on the alliance info?
  int red = 255;
  int blue = 255;

  const double maxIntensityPercent;
  bool breathingIn = true;
  double currentIntensityPercent = 0;
  double increment = 0.01;

  // BUG(Matthew): Why not just store the frc::DriverStation::Alliance value,
  // rather than a "boiled down" case?
  bool isRed = true;
};
