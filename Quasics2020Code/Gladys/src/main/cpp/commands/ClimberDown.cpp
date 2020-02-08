/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberDown.h"

ClimberDown::ClimberDown(Climber*climber) :climber(climber){
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called repeatedly when this Command is scheduled to run
void ClimberDown::Execute() {
  climber->MoveClimberDown();
}

// Called once the command ends or is interrupted.
void ClimberDown::End(bool interrupted) {
  climber->StopClimber();
}

