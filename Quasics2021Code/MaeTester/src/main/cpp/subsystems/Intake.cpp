// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "Constants.h"

Intake::Intake()
    : IntakeMotor(CANBusIds::VictorSPXIds::IntakeMotor),
      ConveyorMotor(CANBusIds::VictorSPXIds::ConveyorMotor) {
  SetSubsystem("Intake");
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

//Intakes the ball at 1/4 speed.
void Intake::IntakeBallOn() {
    IntakeMotor.Set(0.25);

//Stops the intake of ball.
}
void Intake::IntakeBallOff() {
    IntakeMotor.Set(0);

//Reverses the intake at 1/4 speed.
}
void Intake::IntakeBallReverse() {
    IntakeMotor.Set(-0.25);
}
// Runs the conveyor belt of the balls at 1/4 speed.
void Intake::ConveyBallOn() {
  ConveyorMotor.Set(0.25);
}
// Stops the conveyor belt.
void Intake::ConveyBallOff() {
  ConveyorMotor.Set(0);
  // Runs the conveyor belt of the balls in reverse at 1/4 speed.
}
void Intake::ConveyBallReverse() {
  ConveyorMotor.Set(-0.25);
}
