// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoIntakeCells.h"

AutoIntakeCells::AutoIntakeCells(Intake* intake) : intake(intake) {
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void AutoIntakeCells::Initialize() {
  intake->OnlyIntakeOn();
  if (!intake->IsBallInChamber()) {
    intake->ConveyBallOnSlow();
  } else {
    intake->ConveyBallOff();
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoIntakeCells::Execute() {
  intake->OnlyIntakeOn();
  if (!intake->IsBallInChamber()) {
    intake->ConveyBallOnSlow();
  } else {
    intake->ConveyBallOff();
  }
}

// Called once the command ends or is interrupted.
void AutoIntakeCells::End(bool interrupted) {
  intake->IntakeBallOff();
}

// Returns true when the command should end.
bool AutoIntakeCells::IsFinished() {
  return false;
}
