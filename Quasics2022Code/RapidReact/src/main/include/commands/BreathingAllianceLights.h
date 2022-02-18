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

  bool IsFinished() override;

 private:
  Lighting* m_lighting;
  int red = 255;
  int blue = 255;
  const double intensityPercent;
  bool breathingIn = true;
  double currentIntensityPercent = 0;
  double increment = 0.01;
  bool isRed = true;
};
