/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDrive.h"


TankDrive::TankDrive(Drivebase*drivebase, std::function<double()> right, std::function<double()> left)
:drivebase(drivebase), right(right), left(left) { 
  // Use addRequirements() here to declare subsystem dependencies.
AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  drivebase->SetMotorPower(right(), left());
  drivebase->DisplayEncoderValues();
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool TankDrive::IsFinished() { return false; }
