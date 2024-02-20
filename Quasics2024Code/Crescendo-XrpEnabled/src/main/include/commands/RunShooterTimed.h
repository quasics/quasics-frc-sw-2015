// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunShooterTimed
    : public frc2::CommandHelper<frc2::Command, RunShooterTimed> {
 public:
  RunShooterTimed(Shooter &shooter, double shooterSpeed, units::second_t time,
                  bool shooting);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Shooter &m_shooter;
  const double m_shooterSpeed;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
  const bool m_shooting;
};
