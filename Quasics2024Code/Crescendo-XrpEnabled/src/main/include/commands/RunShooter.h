// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

// TODO: (CODE_REVIEW) Add comments.
class RunShooter : public frc2::CommandHelper<frc2::Command, RunShooter> {
 public:
  RunShooter(Shooter& shooter, double shooterSpeed, bool shooting);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Shooter& m_shooter;
  const double m_shooterSpeed;
  const bool m_shooting;
};
