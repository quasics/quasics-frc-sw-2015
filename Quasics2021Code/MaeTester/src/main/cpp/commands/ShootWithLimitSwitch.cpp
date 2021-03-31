// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootWithLimitSwitch.h"

ShootWithLimitSwitch::ShootWithLimitSwitch(Shooter*shooter, Intake*intake) : shooter(shooter), intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ShootWithLimitSwitch::Initialize() {
  shooter->setShootingMotor(.75);
  bool InitialLimitSwitchState = intake->IsBallInChamber();
}

// Called repeatedly when this Command is scheduled to run
void ShootWithLimitSwitch::Execute() {
  //Stage 1
  //Reference time = get current time
  //if ball not in chamber, advance conveyor
  if(!(intake->IsBallInChamber())) {
    intake->ConveyBallOn();
  }
  //Stage 2
  //wait until current time = reference time + X
  //Stage 3
  //feed ball to shooter
  //wait until limit switch is off

}

// Called once the command ends or is interrupted.
void ShootWithLimitSwitch::End(bool interrupted) {
shooter->stopShootingMotor();
intake->ConveyBallOff();

}

// Returns true when the command should end.
bool ShootWithLimitSwitch::IsFinished() {
  return false;
}
