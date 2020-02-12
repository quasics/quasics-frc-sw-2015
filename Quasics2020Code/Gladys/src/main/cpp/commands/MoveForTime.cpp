/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MoveForTime.h"

MoveForTime::MoveForTime(Drivebase*drivebase, double duration, double power)
:drivebase(drivebase), duration(duration), left_power(power), right_power(power){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

MoveForTime::MoveForTime(Drivebase*drivebase, double duration, double left_power, double right_power)
:drivebase(drivebase), duration(duration), left_power(left_power), right_power(right_power){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void MoveForTime::Initialize() {
  tikTok.Start();
  tikTok.Reset();
  drivebase->SetMotorPower(left_power, right_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForTime::Execute() {}

// Called once the command ends or is interrupted.
void MoveForTime::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool MoveForTime::IsFinished() { 
  return tikTok.HasPeriodPassed(units::second_t(duration));
 }
