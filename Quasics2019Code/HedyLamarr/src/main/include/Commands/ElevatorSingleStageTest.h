/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifdef ENABLE_OLD_ELEVATOR
#pragma once

#include <frc/commands/Command.h>
#include "Subsystems/ElevatorStage.h"

class ElevatorSingleStageTest : public frc::Command {
 private:
  ElevatorStage& stage;

 public:
  ElevatorSingleStageTest(ElevatorStage& stage) : stage(stage) {
    Requires(&stage);
  }

  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};
#endif // ENABLE_OLD_ELEVATOR