/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootBallsCommand.h"

ShootBallsCommand::ShootBallsCommand(Exhaust*exhaust):exhaust(exhaust) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShootBallsCommand::Initialize() {
  exhaust -> ShootBallOn();
  exhaust -> PushBallUp();
}

// Called once the command ends or is interrupted.
void ShootBallsCommand::End(bool interrupted) {
  exhaust -> ShootBallOff();
  exhaust -> PushBallOff();
}
