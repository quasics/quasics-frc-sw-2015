/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShoulderControl.h"

ShoulderControl::ShoulderControl(Intake*intake, std::function<double()> upValue)
:intake(intake), upValue(upValue) {
  // Use addRequirements() here to declare subsystem dependencies.

AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ShoulderControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShoulderControl::Execute() {
  if (upValue() > 0) {
    intake -> RotateShoulderUp();
  }
  if (upValue() < 0) {
    intake -> RotateShoulderDown();
  }
  else {
    intake -> TurnShoulderOff();
  }
}

// Called once the command ends or is interrupted.
void ShoulderControl::End(bool interrupted) {

  intake->TurnShoulderOff(); 
}

// Returns true when the command should end.
bool ShoulderControl::IsFinished() { return false; }
