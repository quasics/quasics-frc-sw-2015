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

// Loads balls into the robot from window using the PushBall function.
/// @todo (Nurfadil) Document this class (using JavaDoc format comments).
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LoadFromWindowCommand
    : public frc2::CommandHelper<frc2::CommandBase, LoadFromWindowCommand> {
 public:
  LoadFromWindowCommand(Exhaust* exhaust);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  Exhaust* exhaust;
};
