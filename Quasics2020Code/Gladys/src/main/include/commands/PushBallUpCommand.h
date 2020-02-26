/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Exhaust.h"

// TODO(Nurfadil): Document this class.
// TODO(Nurfadil): Rename this class. to be something more meaningful (or remove
// it).
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PushBallUpCommand
    : public frc2::CommandHelper<frc2::CommandBase, PushBallUpCommand> {
 public:
  PushBallUpCommand(Exhaust* exhaust);

  // Turns on the ball pusher.
  void Initialize() override;

  // Turns off the ball pusher.
  void End(bool interrupted) override;

 private:
  Exhaust* exhaust;
};
