/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DeliverToLowGoalCommand.h"

DeliverToLowGoalCommand::DeliverToLowGoalCommand(Exhaust* exhaust)
    : exhaust(exhaust) {
  AddRequirements(exhaust);
}

// Called when the command is initially scheduled.
void DeliverToLowGoalCommand::Initialize() {
  exhaust->ShootBallDown();
  exhaust->PushBallDown();
}

// Called once the command ends or is interrupted.
void DeliverToLowGoalCommand::End(bool interrupted) {
  exhaust->PushBallOff();
  exhaust->ShootBallOff();
}
