/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootBallsReverseCommand.h"

ShootBallsReverseCommand::ShootBallsReverseCommand(Exhaust* exhaust)
    : exhaust(exhaust) {
  // Use AddRequirements() here to declare subsystem dependencies.
  AddRequirements(exhaust);

  // TODO(Nurfadil): (BUG) Add the climber as a required subsystem!
}

// Called when the command is initially scheduled.
void ShootBallsReverseCommand::Initialize() {
  exhaust->ShootBallOn();
  exhaust->PushBallUp();
}

// Called once the command ends or is interrupted.
void ShootBallsReverseCommand::End(bool interrupted) {
  exhaust->ShootBallOff();
  exhaust->PushBallOff();
}
