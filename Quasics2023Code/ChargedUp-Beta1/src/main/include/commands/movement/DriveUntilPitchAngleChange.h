// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
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
 */

// The robot will drive at a certain speed until it encounters a drastic angle
// change
// contains an optional parameter: robot will stop if it has been driving for
// this distance and has not encountered a radical angle change
class DriveUntilPitchAngleChange
    : public frc2::CommandHelper<frc2::Command,
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
