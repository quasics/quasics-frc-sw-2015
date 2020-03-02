/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwissArmySubsystem.h"

/**
 * A sample command that might interact with one of our subsystems (raising a
 * theoretical elevator).
 *
 * This is provided primarily as a means of demonstrating button binding.
 */
class RaiseElevatorCommand
    : public frc2::CommandHelper<frc2::CommandBase, RaiseElevatorCommand> {
 public:
  RaiseElevatorCommand(SwissArmySubsystem* targetSubsystem);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  SwissArmySubsystem* targetSubsystem;
};
