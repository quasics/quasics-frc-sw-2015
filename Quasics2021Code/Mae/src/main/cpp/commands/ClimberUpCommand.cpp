/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberUpCommand.h"

ClimberUpCommand::ClimberUpCommand(Climber* climber) : climber(climber) {
  // Use AddRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
  
}

// Called repeatedly when this Command is scheduled to run
void ClimberUpCommand::Execute() {
  climber->MoveClimberUp();
}

// Called once the command ends or is interrupted.
void ClimberUpCommand::End(bool interrupted) {
  climber->StopClimber();
}
