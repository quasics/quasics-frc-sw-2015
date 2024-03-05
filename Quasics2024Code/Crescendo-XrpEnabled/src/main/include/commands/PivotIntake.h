// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/dimensionless.h>

#include "subsystems/IntakeDeployment.h"

// TODO: (CODE_REVIEW) Add comments.
class PivotIntake : public frc2::CommandHelper<frc2::Command, PivotIntake> {
 public:
  PivotIntake(IntakeDeployment &IntakeDeployment, double speed, bool extend);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  frc::SlewRateLimiter<units::scalar> m_intakeSlewRateLimiter{1 / 1_s};
  IntakeDeployment &m_intakeDeployment;
  const double m_intakeDeploymentSpeed;
  const bool m_extending;
};
