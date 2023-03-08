// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeDeployment.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Runs the intake deployment motor backwards to lift the intake to safety a.k.a
// inside the robot frame
class RetractIntake
    : public frc2::CommandHelper<frc2::CommandBase, RetractIntake> {
 public:
  RetractIntake(IntakeDeployment* intake, double speed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeDeployment* m_intakeDeployment;
  const double intakeSpeed;
};
