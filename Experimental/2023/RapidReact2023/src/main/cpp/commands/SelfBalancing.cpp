// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"

SelfBalancing::SelfBalancing(Drivebase* drivebase) {
  AddRequirements(drivebase);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  pid.Reset();
  m_drivebase->SetMotorPower(0.4, 0.4);
  pastAngle = m_drivebase->GetAngle();
}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  //Leave Space for the feed forward thing
  units::degree_t currentAngle = m_drivebase->GetAngle();
  double power = 0.0;
  if (noFeedFowardPower == false){
     power = 0.4;
     auto delta = currentAngle - pastAngle;
     if (delta > 2_deg || delta < -2_deg){
       noFeedFowardPower = true;
       activatePID = true;
     }
  }
  if (activatePID){
    power = pid.Calculate(currentAngle.value(), 0.0);
  }

  m_drivebase->SetMotorPower(power, power);
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool SelfBalancing::IsFinished() {
  return false;
}
