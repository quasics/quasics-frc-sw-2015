// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

/**
 * TODO(gavin): Update docs for this class.
 *
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootWithLimitSwitch
    : public frc2::CommandHelper<frc2::CommandBase, ShootWithLimitSwitch> {
 public:
  ShootWithLimitSwitch(Shooter* shooter, Intake* intake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Shooter* shooter;
  Intake* intake;
  int Stage = 0;
  int WaitTime = 3000;
  int ReferenceTime;
};
