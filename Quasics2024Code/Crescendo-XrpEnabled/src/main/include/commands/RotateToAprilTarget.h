// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "commands/TankDrive.h"
#include "subsystems/IDrivebase.h"
#include "subsystems/Vision.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateToAprilTarget
    : public frc2::CommandHelper<frc2::Command, RotateToAprilTarget> {
 public:
  RotateToAprilTarget(IDrivebase& drivebase, Vision& vision, int ID);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Vision& m_vision;
  IDrivebase& m_drivebase;
  const int m_ID;
};
