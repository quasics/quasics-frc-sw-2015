// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/Drivebase.h"

/**
 * Rotates. Positive angle turns left, negative angle turns right
 * Angle should be between -180 and 180.
 * If 270 degrees is inputted, robot will not optimize this to -90
 * degrees.
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
  /** Drive base used to move the robot. */
  Drivebase* const m_drivebase;

  // CODE_REVIEW(ethan): the m_percentSpeed and m_angle values should be
  // "const", and set in the constructor (where they're specified when the
  // object is created).  Recomputing them whenever the command executes is
  // "fragile" coding.
  /** Desired (maximum) speed for the turn. */
  double m_percentSpeed;

  /** Desired amount by which the robot should turn when the command runs. */
  units::degree_t m_angle;

  /**
   * Current angle (yaw) of the robot when the command starts running.  This
   * allows us to determine when the full turn from the starting position has
   * been completed.
   */
  units::degree_t m_startAngle;

  // CODE_REVIEW(ethan): Please document what is this for, and how it is to be
  // used.
  double m_multiplier = 1;
};