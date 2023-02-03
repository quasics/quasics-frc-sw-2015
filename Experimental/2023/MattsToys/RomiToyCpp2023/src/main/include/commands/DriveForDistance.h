// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveForDistance
    : public frc2::CommandHelper<frc2::CommandBase, DriveForDistance>
{
public:
  DriveForDistance(Drivetrain *driveBase, units::inch_t distance)
      : m_driveBase(driveBase), m_distance(distance) { AddRequirements(m_driveBase); }

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Drivetrain *m_driveBase;
  const units::inch_t m_distance;
  units::inch_t m_startingPosition;
  units::inch_t m_endPosition;
};
