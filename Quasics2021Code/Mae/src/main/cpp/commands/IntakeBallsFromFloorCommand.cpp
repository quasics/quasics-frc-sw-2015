/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallsFromFloorCommand.h"

IntakeBallsFromFloorCommand::IntakeBallsFromFloorCommand(Intake*intake,Exhaust*exhaust):intake(intake),exhaust(exhaust){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
  AddRequirements(exhaust);
}

// Called when the command is initially scheduled.
void IntakeBallsFromFloorCommand::Initialize() {
  intake->TurnSuctionOn();
  exhaust->PushBallDown();
}


// Called once the command ends or is interrupted.
void IntakeBallsFromFloorCommand::End(bool interrupted) {
  intake->TurnSuctionOff();
  exhaust->PushBallOff();
}

