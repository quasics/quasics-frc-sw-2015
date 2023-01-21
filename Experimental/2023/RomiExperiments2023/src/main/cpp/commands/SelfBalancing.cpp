// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"

SelfBalancing::SelfBalancing(Drivetrain* drivebase) : m_drivebase(drivebase){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  m_drivebase->ResetGyro();
  noFeedFowardPower = false;
  activatePID = false;
  pid.Reset();
  pastAngle = m_drivebase->GetGyroAngleX();
/*
  if (pastAngle > 0){
    slopeOfRamp = -1;
  }
  else{
    slopeOfRamp = 1;
  }
*/
  m_drivebase->TankDrive(slopeOfRamp*0.4, slopeOfRamp*0.4);

}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  double currentAngle = m_drivebase->GetGyroAngleX();
  double power = 0.0;
  if (noFeedFowardPower == false){
     power = 0.4;
     auto delta = currentAngle - pastAngle;
     if (delta > 2.0 || delta < -2.0){
       noFeedFowardPower = true;
       activatePID = true;
     }
  }
  if (activatePID){
    power = pid.Calculate(currentAngle, 0.0);
  }


/*
  if (pastAngle > 0){
    slopeOfRamp = -1;
  }
  else{
    slopeOfRamp = 1;
  }
  */

  m_drivebase->TankDrive(slopeOfRamp*power, slopeOfRamp*power);
  //pastAngle = currentAngle;
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool SelfBalancing::IsFinished() {
  return false;
}
