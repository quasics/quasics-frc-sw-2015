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

void Climber::MoveClimberUp() {
  std::cout << "Turning climber to 'up'" << std::endl;
  RightClimber.Set(-1);
  LeftClimber.Set(1);
}

void Climber::MoveClimberDown() {
  std::cout << "Turning climber to 'down'" << std::endl;
  RightClimber.Set(1);
  LeftClimber.Set(-1);
}

void Climber::StopClimber() {
  std::cout << "Turning climber off" << std::endl;
  RightClimber.Set(0);
  LeftClimber.Set(0);
  std::cout << "Done with climber" << std::endl;
}