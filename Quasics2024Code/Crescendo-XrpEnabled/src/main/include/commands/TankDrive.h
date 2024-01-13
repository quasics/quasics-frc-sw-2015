// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IDrivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive : public frc2::CommandHelper<frc2::Command, TankDrive> {
 public:
  typedef std::function<double()> PercentSupplier;

 public:
  TankDrive(IDrivebase& driveBase, PercentSupplier leftSupplier,
            PercentSupplier rightSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void updateSpeed();

 private:
  IDrivebase& m_driveBase;
  PercentSupplier m_leftSupplier;
  PercentSupplier m_rightSupplier;
};
