// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IDrivebase.h"

// TODO: (CODE_REVIEW) Add comments.
class ArcadeDrive : public frc2::CommandHelper<frc2::Command, ArcadeDrive> {
 public:
  typedef std::function<double()> PercentSupplier;

  ArcadeDrive(IDrivebase& drivebase, PercentSupplier forwardSupplier,
              PercentSupplier rotationSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  IDrivebase& m_drivebase;
  PercentSupplier m_forwardSupplier;
  PercentSupplier m_rotationSupplier;

  void updateSpeeds();
};
