// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

// TODO: (CODE_REVIEW) Add comments.
class SetLightsToColor
    : public frc2::CommandHelper<frc2::Command, SetLightsToColor> {
 public:
  SetLightsToColor(Lighting* lighting, int r, int g, int b);

  void Execute() override;

 private:
  Lighting* m_lighting;
  const int m_r;
  const int m_g;
  const int m_b;
};
