// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeDeployment.h"

// TODO: (CODE_REVIEW) Add comments.
class PivotIntakeAuto
    : public frc2::CommandHelper<frc2::Command, PivotIntakeAuto> {
 public:
  PivotIntakeAuto(IntakeDeployment& IntakeDeployment, double speed,
                  bool extend);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc::SlewRateLimiter<units::dimensionless::scalar> m_intakeSlewRateLimiter{
      .5 / 1_s};
  IntakeDeployment& m_intakeDeployment;
  const double m_intakeDeploymentSpeed;
  const bool m_extending;
};
