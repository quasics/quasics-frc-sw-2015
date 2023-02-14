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
 *  // m_distance can be positive (forwards) or negative (backwards),
 m_motorPower
    // will always be positive after this code even if user enters weird values.
 */
class DriveAtPowerForMeters
    : public frc2::CommandHelper<frc2::CommandBase, DriveAtPowerForMeters> {
 public:
  DriveAtPowerForMeters(Drivebase* drivebase, double motorPower,
                        units::meter_t distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* m_drivebase;
  double m_motorPower;
  units::meter_t m_distance;
  units::meter_t m_leftStartingPosition;
  units::meter_t m_rightStartingPosition;
  double m_multiplier;
};
