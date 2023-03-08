// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * Changes the color of the entire light strip using RGB data.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * CODE_REVIEW(rylie): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.
 */
class SetLightsToColor
    : public frc2::CommandHelper<frc2::CommandBase, SetLightsToColor> {
 public:
  SetLightsToColor(Lighting* lighting, int r, int g, int b);

  void Execute() override;

 private:
  Lighting* m_lighting;
  int m_r;
  int m_g;
  int m_b;
};
