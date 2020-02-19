/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LoadFromWindowCommand.h"

LoadFromWindowCommand::LoadFromWindowCommand(Exhaust*exhaust):exhaust(exhaust) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void LoadFromWindowCommand::Initialize() {
  //pushes ball into storage from window
  exhaust->PushBallDown();
}


// Called once the command ends or is interrupted.
void LoadFromWindowCommand::End(bool interrupted) {
  //turns off loading motor
  exhaust->PushBallOff();
}

