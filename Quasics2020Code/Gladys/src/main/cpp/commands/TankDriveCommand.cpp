/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDriveCommand.h"

TankDriveCommand::TankDriveCommand(Drivebase* drivebase,
                                   std::function<double()> right,
                                   std::function<double()> left)
    : drivebase(drivebase), right(right), left(left) {
  AddRequirements(drivebase);
}

// TODO(Kat): Remove this unneeded function (from both .cpp and .h).
// Called when the command is initially scheduled.
void TankDriveCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void TankDriveCommand::Execute() {
  drivebase->SetMotorPower(right(), left());
  drivebase->DisplayEncoderValues();
}

// Called once the command ends or is interrupted.
void TankDriveCommand::End(bool interrupted) {
  drivebase->Stop();
}

// TODO(Kat): Remove this unneeded function (from both .cpp and .h).
// Returns true when the command should end.
bool TankDriveCommand::IsFinished() {
  return false;
}
