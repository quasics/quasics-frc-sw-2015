// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"
#include <iostream>
SelfBalancing::SelfBalancing(Drivetrain* drivebase) : m_drivebase(drivebase){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  noFeedFowardPower = false;
  activatePID = false;
  pid.Reset();
  pastAngle = m_drivebase->GetGyroAngleY();
//added a minus 1 to compensate for negative deviations
  if ((pastAngle) > 0){
    slopeOfRamp = 1;
  }
  if(pastAngle < 0){
    slopeOfRamp = -1;
  }

  m_drivebase->TankDrive(slopeOfRamp, slopeOfRamp);

}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  //PAST ANGLE READING ADJUSTEMENT FOR ACCURATE RESULTS
  std::cout <<"Current Gyro Reading: " << (pastAngle) << std::endl;
  //gyro not reading in degrees, going from 0 to 9
  //also resets the gyro so its current position is 0
  double currentAngle = m_drivebase->GetGyroAngleY();
  double power = 0.0;
  if (noFeedFowardPower == false){
     power = 1;
     auto delta = currentAngle - pastAngle;
     if (currentAngle > -2.5 and currentAngle < 2.5){
     //TEMPORARY FIX TO TEST OTHER CODE
     //if (false){
       noFeedFowardPower = true;
       activatePID = true;
     }
  }
  /*
    OHHHHH I GET IT THIS IS THE REASON IT WAS FAILING:
      pid accounts for direction, so when the romi was going backward
      and the slope was -1 then a double negation occured which made 
      the romi continue going in the same direction. As the Romi got 
      futher away the PID correction only increased which was the
      reason why the Romi speed dramatically increased in 1 direction
  
  
  
  
  
  
  */
  if (activatePID){
    power = pid.Calculate(currentAngle, 0.0);
  }


//added a minus 1 to compensate for negative deviations
  if ((pastAngle) > 0){
    slopeOfRamp = 1;
  }
  if(pastAngle < 0){
    slopeOfRamp = -1;
  }
  
  //SOLVED THE ISSUE I THINK. SORRY ABOUT THAT
  if(!activatePID){
    m_drivebase->TankDrive(slopeOfRamp*power, slopeOfRamp*power);
  }
  if(activatePID){
    m_drivebase->TankDrive(power, power);
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
