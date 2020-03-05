/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShoulderControlCommand.h"

ShoulderControlCommand::ShoulderControlCommand(Intake* intake,
                                               std::function<double()> upValue)
    : intake(intake), upValue(upValue) {
  AddRequirements(intake);
}

/// @todo (Nurfadil) Remove unneeded function (from .cpp and .h).
// Called when the command is initially scheduled.
void ShoulderControlCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ShoulderControlCommand::Execute() {
  const double currentValue = upValue();
  std::cout << "Value in command: " << currentValue << std::endl;
  if (currentValue > 0) {
    intake->RotateShoulderUp();
  } else if (currentValue < 0) {
    intake->RotateShoulderDown();
  } else {
    intake->TurnShoulderOff();
  }
}

// Called once the command ends or is interrupted.
void ShoulderControlCommand::End(bool interrupted) {
  intake->TurnShoulderOff();
}

/// @todo (Nurfadil) Remove unneeded function (from .cpp and .h).
// Returns true when the command should end.
bool ShoulderControlCommand::IsFinished() {
  return false;
}
