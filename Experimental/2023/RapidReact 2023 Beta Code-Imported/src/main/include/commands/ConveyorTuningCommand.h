// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableEntry.h>

#include "subsystems/Conveyor.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ConveyorTuningCommand
    : public frc2::CommandHelper<frc2::CommandBase, ConveyorTuningCommand> {
 public:
  ConveyorTuningCommand(Conveyor* conveyor, double initialConveyorSpeedPercent);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Conveyor* m_conveyor;
  nt::NetworkTableEntry m_conveyorSpeedSlider;
};
