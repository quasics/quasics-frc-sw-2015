/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Climber.h"

#include "Constants.h"

Climber::Climber()
    : RightClimber(CANBusIds::VictorSpx::RightClimberNumber),
      LeftClimber(CANBusIds::VictorSpx::LeftClimberNumber) {
  SetSubsystem("Climber");
}

// This method will be called once per scheduler run
void Climber::Periodic() {
}

void Climber::MoveClimberUp() {
  RightClimber.Set(-1);
  LeftClimber.Set(1);
}

void Climber::MoveClimberDown() {
  RightClimber.Set(1);
  LeftClimber.Set(-1);
}

void Climber::StopClimber() {
  RightClimber.Set(0);
  LeftClimber.Set(0);
}