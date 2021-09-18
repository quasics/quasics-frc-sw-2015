// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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
class DriveAtPowerForMeters
    : public frc2::CommandHelper<frc2::CommandBase, DriveAtPowerForMeters> {
 public:
  /**
   * Drives forard for a certin amount of meters at a certain amount of power
   *
   * @param drivebase pointer to the drive base
   * @param power the % power (for speed control, e.g., 0.5 for 50% speed).
   *              This will be treated as an absolute value, so sign is ignored.
   * @param desiredMeters the Distance to drive (e.g., 3_m to drive 3 meters
   *                      forward, -2_ft to drive 2 feet backward)
   */
  DriveAtPowerForMeters(Drivebase* drivebase, double power,
                        units::length::meter_t desiredMeters);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  units::length::meter_t startingMeters;
  units::length::meter_t destination;
  Drivebase* drivebase;
  const double power;
  const units::length::meter_t desiredMeters;
};
