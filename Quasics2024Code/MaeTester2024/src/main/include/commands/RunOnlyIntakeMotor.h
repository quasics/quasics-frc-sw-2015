// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

/**
 * TODO(scott): Update docs for this class.
 *
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunOnlyIntakeMotor
    : public frc2::CommandHelper<frc2::Command, RunOnlyIntakeMotor> {
 public:
  RunOnlyIntakeMotor(Intake* intake);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  Intake* intake;
};
