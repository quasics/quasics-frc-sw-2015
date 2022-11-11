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
class RotateOnArc
    : public frc2::CommandHelper<frc2::CommandBase, RotateOnArc> {
 public:
  RotateOnArc(DriveBase* driveBase, bool turnLeft, double percentSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    DriveBase* m_driveBase;
    bool m_turnLeft;
    double m_percentSpeed;
    int m_degreesTurned;

};
