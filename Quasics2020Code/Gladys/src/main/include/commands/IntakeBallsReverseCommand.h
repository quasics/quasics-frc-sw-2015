/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"
//
// TODO(Nurfadil): Document this class.
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeBallsReverseCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeBallsReverseCommand> {
 public:
  IntakeBallsReverseCommand(Intake* intake);

  // Reverses the ball intaker.
  void Initialize() override;

  // Stops ball intake rotation.
  void End(bool interrupted) override;

 private:
  Intake* intake;
};
