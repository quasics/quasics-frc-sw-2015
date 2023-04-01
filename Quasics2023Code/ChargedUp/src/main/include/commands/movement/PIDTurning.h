// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
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
class PIDTurning : public frc2::CommandHelper<frc2::CommandBase, PIDTurning> {
 public:
  PIDTurning(Drivebase* drivebase, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  void FeedForward();

 private:
  Drivebase* m_drivebase;
  const units::degree_t m_angle;
  frc2::PIDController pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
                          PIDTurningConstants::kD};
  units::degree_t startingAngle = 0_deg;
  units::degree_t currentAngle = 0_deg;
  bool feedForward = true;
  bool activatePID = false;
  double rotationCorrection = 0;
};
