/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveADistanceCommand.h"

#include <cmath>

#include "utils/EncoderHelpers.h"

DriveADistanceCommand::DriveADistanceCommand(Drivebase* drivebase,
                                             double distance, double power)
    : DriveADistanceCommand(drivebase, distance, power, power) {
}

DriveADistanceCommand::DriveADistanceCommand(Drivebase* drivebase,
                                             double distance, double leftPower,
                                             double rightPower)
    : drivebase(drivebase),
      distance(std::abs(distance)),
      leftPower(leftPower),
      rightPower(rightPower) {
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveADistanceCommand::Initialize() {
  drivebase->ResetEncoderPositions();
  drivebase->SetMotorPower(leftPower, rightPower);
}

// Called repeatedly when this Command is scheduled to run
void DriveADistanceCommand::Execute() {
}

// Called once the command ends or is interrupted.
void DriveADistanceCommand::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool DriveADistanceCommand::IsFinished() {
  const double leftEncoderDistance = std::abs(drivebase->GetLeftEncoderInInches());
  const double rightEncoderDistance = std::abs(drivebase->GetRightEncoderInInches());
  
  if ((leftEncoderDistance >= distance) || (rightEncoderDistance >= distance)) {
    return true;
  }
  return false;
}
