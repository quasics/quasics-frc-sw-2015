// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>
#include "subsystems/DriveBase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveInALine
    : public frc2::CommandHelper<frc2::CommandBase, MoveInALine> {
 public:
  MoveInALine(DriveBase* driveBase, units::meter_t distanceToTravel, double percentSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  DriveBase* m_driveBase;
  units::meter_t m_distanceToTravel;
  double m_percentSpeed;
  units::meter_t m_startPosition;
};
