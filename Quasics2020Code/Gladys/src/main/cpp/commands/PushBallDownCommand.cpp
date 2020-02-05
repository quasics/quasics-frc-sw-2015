/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PushBallDownCommand.h"

PushBallDownCommand::PushBallDownCommand(Exhaust*exhaust):exhaust(exhaust) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PushBallDownCommand::Initialize() {
  exhaust -> PushBallDown();
}

// Called once the command ends or is interrupted.
void PushBallDownCommand::End(bool interrupted) {
  exhaust -> PushBallOff();
}

