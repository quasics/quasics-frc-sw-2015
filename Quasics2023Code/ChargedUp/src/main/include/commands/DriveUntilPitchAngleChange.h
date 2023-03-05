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
 *
 * CODE_REVIEW(matthew): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.
 */
class DriveUntilPitchAngleChange
    : public frc2::CommandHelper<frc2::CommandBase,
                                 DriveUntilPitchAngleChange> {
 public:
  // Note that this constructor will normalize the input values provided.
  // If distance and/or speed is negative, then we assume were going backward
  // for abs(distance) meters
  DriveUntilPitchAngleChange(Drivebase* drivebase, double power,
                             units::meter_t distance = 10_m);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* m_drivebase;
  const double m_power;
  const units::meter_t m_distance;
  units::meter_t m_leftStartingPosition;
  units::meter_t m_rightStartingPosition;
};
