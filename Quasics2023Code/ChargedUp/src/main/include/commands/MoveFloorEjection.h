// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FloorEjection.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Given a speed will move the floor in a certain direction to rid the robot of
// the game piece
class MoveFloorEjection
    : public frc2::CommandHelper<frc2::CommandBase, MoveFloorEjection> {
 public:
  MoveFloorEjection(FloorEjection* floorEjection, double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  FloorEjection* m_floorEjection;
  const double m_power;
};
