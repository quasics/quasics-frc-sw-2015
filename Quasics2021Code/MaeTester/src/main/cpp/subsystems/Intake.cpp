// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "Constants.h"

Intake::Intake() : IntakeMotor(CANBusIds::VictorSPXIds::IntakeMotor){}

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

