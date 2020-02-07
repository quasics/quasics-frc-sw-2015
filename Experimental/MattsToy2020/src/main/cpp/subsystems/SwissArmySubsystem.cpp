/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwissArmySubsystem.h"

SwissArmySubsystem::SwissArmySubsystem() {
}

// This method will be called once per scheduler run
void SwissArmySubsystem::Periodic() {
}

void SwissArmySubsystem::RotateShoulderDown() {
  shoulder.Set(0.25);
}

void SwissArmySubsystem::RotateShoulderUp() {
  shoulder.Set(-0.25);
}

void SwissArmySubsystem::StopMovingShoulder() {
  shoulder.Set(0);
}

void SwissArmySubsystem::SetElevatorPower(double left, double right) {
  this->leftElevatorMotor.Set(left);
  this->rightElevatorMotor.Set(right);
}
