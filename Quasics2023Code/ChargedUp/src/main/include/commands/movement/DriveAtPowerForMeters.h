// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "subsystems/Drivebase.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * CODE_REVIEW(matthew): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.  You should also indicate how this differs from
 * "DriveAtPowerForMeters", and why they're both available.
 */
class DriveAtPowerForMeters
    : public frc2::CommandHelper<frc2::CommandBase,
                                 DriveAtPowerForMeters> {
 public:
  DriveAtPowerForMeters(Drivebase* drivebase, double motorPower,
                                      units::meter_t distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* const m_drivebase;
  const double m_motorPower;
  const units::meter_t m_distance;

  // CODE_REVIEW(matthew): The names don't make it clear what some of these
  // variables do (e.g., "m_subtraction").  Please either update the names to
  // better reflect purpose, or add comments to clarify (or both).
  frc::SlewRateLimiter<units::scalar> m_slewRateLimiter{0.5 / 1_s};
  bool m_accelerating = true;
  units::meter_t m_originalDistance = 0_m;
  units::meter_t m_currentDistance = 0_m;
  units::meter_t m_distanceToDestination = 0_m;
  double m_subtraction = 0;
  double m_gradualreduction = 0.5;
  int m_counter = 0;
};
