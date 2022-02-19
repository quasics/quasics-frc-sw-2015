// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * Command to make the robot's lights "breath" (cycle between dark and light)
 * with a given color and maximum intensity (brightness).
 */
class BreathingAllianceLights
    : public frc2::CommandHelper<frc2::CommandBase, BreathingAllianceLights> {
 public:
  // Constructor.
  BreathingAllianceLights(Lighting* lights, double intensity);

  // Standard command functions.
 public:
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  // Data members.
 private:
  Lighting* m_lighting;
  // BUG(Matthew): Why have 2 variables here, especially when they're just
  // placeholders for the same constant (255/full intensity) in each case?
  // Why not just have one constant, and "pop it into" the right point of
  // the triplet, based on the alliance info?
  int red = 255;
  int blue = 255;

  const double m_maxIntensityPercent;
  double m_currentIntensityPercent = 0;

  // Bug(Matthew): Why do you need both of these?  If the increment is
  // positive (or negative),then you know your breathing in (or out).
  // Alternatively, if you know you're breathing in (or out) then you
  // know that you're incrementing (or decrementing) the current intensity.
  bool m_breathingIn = true;
  double m_increment = 0.01;

  // BUG(Matthew): Why not just store the frc::DriverStation::Alliance value,
  // rather than a "boiled down" case?
  bool m_isRed = true;
};
