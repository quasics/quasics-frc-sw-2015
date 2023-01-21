// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"

SelfBalancing::SelfBalancing(Drivetrain* drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  pid.Reset();
  m_drivebase->TankDrive(0.4, 0.4);
  pastAngle = m_drivebase->GetGyroAngleX();
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

  m_drivebase->TankDrive(power, power);
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool SelfBalancing::IsFinished() {
  return false;
}
