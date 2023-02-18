// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * Rotates. Positive angle turns left, negative angle turns right
 *
 *
 * Angle should be between -180 and 180 degrees to be optimized.
 * If 270 degrees is inputted, robot will not optimize this to -90 degrees
 */
class RotateAtAngle
    : public frc2::CommandHelper<frc2::CommandBase, RotateAtAngle> {
 public:
  RotateAtAngle(Drivebase* drivebase, double percentSpeed,
                units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* m_drivebase;
  double m_percentSpeed;
  units::degree_t m_angle;
  units::degree_t m_startAngle;
  double m_multiplier = 1;
};