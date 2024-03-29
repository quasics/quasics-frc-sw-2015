// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeRoller.h"

// TODO: Add comments to document this class.
class RunIntakeTimed
    : public frc2::CommandHelper<frc2::Command, RunIntakeTimed> {
 public:
  RunIntakeTimed(IntakeRoller &intake, double intakeSpeed,
                 units::second_t timeToRunIntake, bool takingIn);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeRoller &m_intake;
  const double m_intakeSpeed;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
  const bool m_takingIn;
};
