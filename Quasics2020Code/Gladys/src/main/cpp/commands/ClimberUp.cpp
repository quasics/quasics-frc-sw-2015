/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberUp.h"

ClimberUp::ClimberUp(Climber*climber) :climber(climber){
  // Use addRequirements() here to declare subsystem dependencies.
}


// Called repeatedly when this Command is scheduled to run
void ClimberUp::Execute() {
  climber->MoveClimberUp();
}

// Called once the command ends or is interrupted.
void ClimberUp::End(bool interrupted) {
  climber->StopClimber();
}


