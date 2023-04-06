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
 */
class DriveAtPowerForMetersWorkingVersion
    : public frc2::CommandHelper<frc2::CommandBase,
                                 DriveAtPowerForMetersWorkingVersion> {
 public:
  DriveAtPowerForMetersWorkingVersion(Drivebase* drivebase, double motorPower,
                                      units::meter_t distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* const m_drivebase;
  double m_motorPower;
  units::meter_t m_distance;

  frc::SlewRateLimiter<units::scalar> SlewRateLimiter{0.5 / 1_s};
  bool accelerating = true;

  double robotSpeed = 0;
  units::meter_t originalDistance = 0_m;
  units::meter_t currentDistance = 0_m;
  units::meter_t distanceToDestination = 0_m;
  double subtraction = 0;
  double gradualreduction = 0.5;
  int counter = 0;
};
