// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climber.h"

// TODO: (CODE_REVIEW) Add comments.
class MoveClimbers : public frc2::CommandHelper<frc2::Command, MoveClimbers> {
 public:
  MoveClimbers(Climber &climber, bool extending);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Climber &m_climber;
  const bool m_extending;
};
