/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/Timer.h>
#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveForTime
    : public frc2::CommandHelper<frc2::CommandBase, MoveForTime> {
 public:
  MoveForTime(Drivebase*drivebase, double duration, double power);
  MoveForTime(Drivebase*drivebase, double duration, double left_power, double right_power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Drivebase*drivebase;
  frc2::Timer tikTok;
  double duration;
  double left_power;
  double right_power;
};
