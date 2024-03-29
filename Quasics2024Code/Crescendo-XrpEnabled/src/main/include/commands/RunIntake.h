// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeRoller.h"

// TODO: (CODE_REVIEW) Add comments.
class RunIntake : public frc2::CommandHelper<frc2::Command, RunIntake> {
 public:
  RunIntake(IntakeRoller &intake, double intakeSpeed, bool takingIn);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  IntakeRoller &m_intake;
  const double m_intakeSpeed;
  const bool m_takingIn;
};
