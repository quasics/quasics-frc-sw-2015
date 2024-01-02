// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IDrivebase.h"

class ArcadeDriveCommand
    : public frc2::CommandHelper<frc2::Command, ArcadeDriveCommand> {
 public:
  typedef std::function<double()> PercentSupplier;

 private:
  IDrivebase& m_drivebase;
  PercentSupplier m_forwardSupplier;
  PercentSupplier m_rotationSupplier;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

 public:
  ArcadeDriveCommand(IDrivebase& drivebase, PercentSupplier forwardSupplier,
                     PercentSupplier rotationSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void updateSpeeds();
};
