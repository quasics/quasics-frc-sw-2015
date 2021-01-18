// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Drivetrain.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveForward
    : public frc2::CommandHelper<frc2::CommandBase, DriveForward>
{
public:
  DriveForward(Drivetrain *driveTrain, double power) : m_driveTrain(driveTrain), m_power(power)
  {
    AddRequirements({m_driveTrain});
  }

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

private:
  Drivetrain *const m_driveTrain;
  double m_power;
};
