// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetLightsToColor
    : public frc2::CommandHelper<frc2::Command, SetLightsToColor> {
 public:
  SetLightsToColor(Lighting* lighting, int r, int g, int b);

  void Execute() override;

  bool IsFinished() override;

 private:
  Lighting* m_lighting;
  int m_r;
  int m_g;
  int m_b;
};
