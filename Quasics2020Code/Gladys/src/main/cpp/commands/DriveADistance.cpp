/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// There are 42 encoder ticks in a revolution. (The output from .GetPosition()
  // is in ticks)
  // formula for encoder value to inches: (encoder output in ticks)/(42
  // ticks)/(10.71 revolutions)*(6Pi inches forward)


#include "commands/DriveADistance.h"

#include "utils/EncoderHelpers.h"

DriveADistance::DriveADistance(Drivebase*drivebase, double distance, double power) 
  : drivebase(drivebase),
    distance(distance),
    leftPower(power),
    rightPower(power) {
  AddRequirements(drivebase);
}
DriveADistance::DriveADistance(Drivebase*drivebase, double distance, double leftPower, double rightPower) 
  : drivebase(drivebase),
    distance(distance),
    leftPower(leftPower),
    rightPower(rightPower) {
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveADistance::Initialize() {
drivebase->ResetEncoderPositions();
drivebase->SetMotorPower(leftPower, rightPower);


}

// Called repeatedly when this Command is scheduled to run
void DriveADistance::Execute() {
}

// Called once the command ends or is interrupted.
void DriveADistance::End(bool interrupted) {
  drivebase->Stop(); 
}

// Returns true when the command should end.
bool DriveADistance::IsFinished() { 
  if (drivebase->GetLeftEncoderInInches() >= distance && drivebase->GetRightEncoderInInches() >= distance){
    return true;
  }
    return false; 
  }
 
