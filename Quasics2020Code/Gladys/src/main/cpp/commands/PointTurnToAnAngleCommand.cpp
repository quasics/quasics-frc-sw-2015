/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PointTurnToAnAngleCommand.h"
#include "utils/MathHelpers.h"
#include <cmath>

PointTurnToAnAngleCommand::PointTurnToAnAngleCommand(Drivebase * drivebase, bool isTurningRight, double angle):
 drivebase(drivebase),
 isTurningRight (isTurningRight),
 angle(angle)
 {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void PointTurnToAnAngleCommand::Initialize() {
  drivebase->ResetEncoderPositions();
}

// Called repeatedly when this Command is scheduled to run
void PointTurnToAnAngleCommand::Execute() {
  if(isTurningRight){
    drivebase->SetMotorPower(.2, -.2);
  }
  else{
    drivebase->SetMotorPower(-.2, .2);
  }
}

// Called once the command ends or is interrupted.
void PointTurnToAnAngleCommand::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool PointTurnToAnAngleCommand::IsFinished() {
  if(MathHelpers::inchesToDegreesConverter(std::abs(drivebase->GetLeftEncoderInInches())) >= angle){
    return true;
  }
  return false;
}
