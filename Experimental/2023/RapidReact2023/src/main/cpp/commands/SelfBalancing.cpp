// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"
#include <iostream>

SelfBalancing::SelfBalancing(Drivebase* drivebase) : m_drivebase(drivebase){
  AddRequirements(drivebase);
  // Use addRequirements() here to declare subsystem dependencies.
}
/*
ADDED A CALIBRATION GYRO FUNCTION TO DRIVEBASE SUBSYSTEM
*/
// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  noFeedFowardPower = false;
  activatePID = false;
  pid.Reset();
  pid.SetTolerance(2.5, 0);
  pastAngle = m_drivebase->GetPitch();
  if ((pastAngle) > 0){
    slopeOfRamp = 1;
  }
  if(pastAngle < 0){
    slopeOfRamp = -1;
  }

  m_drivebase->SetMotorPower(slopeOfRamp*0.4, slopeOfRamp*0.4);

}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  std::cout <<"Current Gyro Reading: " << (pastAngle) << std::endl;
  double currentAngle = m_drivebase->GetPitch();
  double power = 0.0;
  if (noFeedFowardPower == false){
     power = 0.4;
     if (currentAngle > -2.5 and currentAngle < 2.5){
       noFeedFowardPower = true;
       activatePID = true;
     }
  }
  if (activatePID){
    power = pid.Calculate(currentAngle, 0.0);
  }

  if ((pastAngle) > 0){
    slopeOfRamp = 1;
  }
  if(pastAngle < 0){
    slopeOfRamp = -1;
  }
  
  if(!activatePID){
    m_drivebase->SetMotorPower(slopeOfRamp*power, slopeOfRamp*power);
  }
  if(activatePID){
    m_drivebase->SetMotorPower(power, power);
  }
  pastAngle = currentAngle;
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool SelfBalancing::IsFinished() {
  return false;
}
