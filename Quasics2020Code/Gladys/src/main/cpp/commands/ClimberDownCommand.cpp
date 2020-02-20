/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberDownCommand.h"

ClimberDownCommand::ClimberDownCommand(Climber* climber) : climber(climber) {
  // Use AddRequirements() here to declare subsystem dependencies.

  // TODO(Gavin): (BUG) Add the climber as a required subsystem!
}

// Called repeatedly when this Command is scheduled to run
void ClimberDownCommand::Execute() {
  climber->MoveClimberDown();
}

// Called once the command ends or is interrupted.
void ClimberDownCommand::End(bool interrupted) {
  climber->StopClimber();
}
