// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/LinearActuators.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveLinearActuators
    : public frc2::CommandHelper<frc2::Command, MoveLinearActuators> {
 public:
  MoveLinearActuators(LinearActuators& LinearActuators, bool exetending);

  void Initialize() override;

  bool IsFinished() override;

 private:
  LinearActuators& m_linearActuators;
  bool m_extending;
};
