// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunMotor.h"

RunMotor::RunMotor(MotorOne* motorOne, double power) : motorOne(motorOne), power(power){

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({motorOne});
}

// Called when the command is initially scheduled.
void RunMotor::Initialize() {
  motorOne-> SetSpeed(power);
}


// Called once the command ends or is interrupted.
void RunMotor::End(bool interrupted) {
  motorOne-> Off();
}

