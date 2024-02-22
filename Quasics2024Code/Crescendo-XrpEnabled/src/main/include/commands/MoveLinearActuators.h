// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/LinearActuators.h"

// TODO: (CODE_REVIEW) Add comments.
class MoveLinearActuators
    : public frc2::CommandHelper<frc2::Command, MoveLinearActuators> {
 public:
  MoveLinearActuators(LinearActuators& LinearActuators, bool exetending);

  void Initialize() override;

 private:
  LinearActuators& m_linearActuators;
  bool m_extending;
};
