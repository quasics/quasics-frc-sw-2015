// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivebase.h"
#include <units/angle.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateAtAngle
    : public frc2::CommandHelper<frc2::CommandBase, RotateAtAngle> {
 public:
  RotateAtAngle(Drivebase* drivebase, double percentSpeed, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* m_drivebase;
  double m_percentSpeed;
  units::degree_t m_angle;
  units::degree_t m_startAngle;
  double multiplier = 1.0; // slow down towards end of rotation so robot doesn't overshoot

};