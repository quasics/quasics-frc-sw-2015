/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ShootFrisbee.h"

ShootFrisbee::ShootFrisbee() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::shooter.get());
}

// Called just before this Command runs the first time
void ShootFrisbee::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootFrisbee::Execute() {
  if(Robot::oi->ShooterIsTriggered()){
    Robot::shooter->SetShooterMotors(1);
  }
  else if(!Robot::oi->ShooterIsTriggered()){
    Robot::shooter->SetShooterMotors(0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ShootFrisbee::IsFinished() { return false; }

// Called once after isFinished returns true
void ShootFrisbee::End() {
  Robot::shooter->SetShooterMotors(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootFrisbee::Interrupted() {
  End();
}
