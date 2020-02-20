/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DeliverToLowGoalCommand.h"

DeliverToLowGoalCommand::DeliverToLowGoalCommand(Exhaust* exhaust)
    : exhaust(exhaust) {
  // Use AddRequirements() here to declare subsystem dependencies.

  // TODO(Nurfadil): (BUG) Add the climber as a required subsystem!
}

// Called when the command is initially scheduled.
void DeliverToLowGoalCommand::Initialize() {
  exhaust->PushBallDown();
  exhaust->ShootBallOn();
}

// Called once the command ends or is interrupted.
void DeliverToLowGoalCommand::End(bool interrupted) {
  exhaust->PushBallOff();
  exhaust->ShootBallOff();
}
