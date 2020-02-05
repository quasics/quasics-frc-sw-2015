/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallsCommand.h"

IntakeBallsCommand::IntakeBallsCommand(Intake*intake):intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeBallsCommand::Initialize() {
  intake -> TurnSuctionOn();
}

// Called once the command ends or is interrupted.
void IntakeBallsCommand::End(bool interrupted) {
  intake -> TurnSuctionOff();
}

