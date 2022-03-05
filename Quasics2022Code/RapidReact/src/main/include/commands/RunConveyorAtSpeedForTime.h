// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Conveyor.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunConveyorAtSpeedForTime
    : public frc2::CommandHelper<frc2::CommandBase, RunConveyorAtSpeedForTime> {
 public:
  RunConveyorAtSpeedForTime(Conveyor* conveyor, double speed,
                            units::second_t time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Conveyor* m_conveyor;
  const double m_conveyorSpeed;
  frc::Timer m_stopWatch;
  const units::second_t m_time;
};
