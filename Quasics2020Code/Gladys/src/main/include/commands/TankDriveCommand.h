/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/// TODO(Kat): Document this class (using JavaDoc format comments).
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, TankDriveCommand> {
 public:
  TankDriveCommand(Drivebase* drivebase, std::function<double()> right,
                   std::function<double()> left);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Drivebase* drivebase;
  std::function<double()> right;
  std::function<double()> left;
};
