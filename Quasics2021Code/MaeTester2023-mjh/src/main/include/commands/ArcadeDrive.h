// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArcadeDrive : public frc2::CommandHelper<frc2::CommandBase, ArcadeDrive> {
 public:
  ArcadeDrive(Drivebase* drivebase, std::function<double()> powerFunction,
              std::function<double()> turnFunction);

  void Initialize() override { UpdateSpeeds(); }

  void Execute() override { UpdateSpeeds(); }

  void End(bool interrupted) override { m_drivebase->Stop(); }

 private:
  void UpdateSpeeds();

 private:
  Drivebase* m_drivebase;
  std::function<double()> m_powerFunction;
  std::function<double()> m_turnFunction;
};
