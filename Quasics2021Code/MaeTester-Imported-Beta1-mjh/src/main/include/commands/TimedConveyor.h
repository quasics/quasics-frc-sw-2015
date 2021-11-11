// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h> // maybe
#include <frc/Timer.h>

#include "subsystems/Intake.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TimedConveyor
    : public frc2::CommandHelper<frc2::CommandBase, TimedConveyor>
{
public:
  TimedConveyor(Intake *intake, units::second_t time, bool forward);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Intake *intake;
  const units::second_t time;
  const bool forward;
  frc::Timer stopWatch;
};
