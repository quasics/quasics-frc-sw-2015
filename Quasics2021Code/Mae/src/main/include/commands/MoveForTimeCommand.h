/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/// @todo (Scott) Consider deleting this class, now that we have distance-based
// movement control.  Either that, or finish documenting its members.
/**
 * Moves the robot at a certain percent power over a certain time.  (Intended as
 * a first approximation of movement support required for autonomous mode; needs
 * to be replaced by moving for distance.)
 */
class MoveForTimeCommand
    : public frc2::CommandHelper<frc2::CommandBase, MoveForTimeCommand> {
 public:
  MoveForTimeCommand(Drivebase* drivebase, double duration, double power);
  MoveForTimeCommand(Drivebase* drivebase, double duration, double left_power,
                     double right_power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* drivebase;
  frc2::Timer tikTok;
  double duration;
  double left_power;
  double right_power;
};
