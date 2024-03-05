// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lighting.h"

// TODO: (CODE_REVIEW) Add comments.
class MatchPlayLighting
    : public frc2::CommandHelper<frc2::Command, MatchPlayLighting> {
 public:
  MatchPlayLighting(Lighting* lighting);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Lighting* m_lighting;
};
