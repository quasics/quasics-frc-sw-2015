// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunOnlyConveyorMotorReverse.h"

RunOnlyConveyorMotorReverse::RunOnlyConveyorMotorReverse(Intake* intake)
    : intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void RunOnlyConveyorMotorReverse::Initialize() {
  intake->ConveyBallReverse();
}

// Called once the command ends or is interrupted.
void RunOnlyConveyorMotorReverse::End(bool interrupted) {
  intake->ConveyBallOff();
}
