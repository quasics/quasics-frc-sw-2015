// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include <frc/DigitalInput.h>

#include "Constants.h"

Intake::Intake()
    : IntakeMotor(CANBusIds::VictorSPXIds::IntakeMotor),
      ConveyorMotor(CANBusIds::VictorSPXIds::ConveyorMotor) {
  ConveyorLimitSwitch.reset(new frc::DigitalInput(0));
  SetSubsystem("Intake");
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

// Intakes the ball at 1/4 speed. Theoretically, when limit switch is hit, both
// Intake and Conveyor will stop.
void Intake::IntakeBallOn() {
  if (ConveyorLimitSwitch.get()) {
    ConveyorMotor.Set(0);
    IntakeMotor.Set(0);
  } else {
    IntakeMotor.Set(0.75);
    ConveyorMotor.Set(0.25);
  }

//Stops the intake of ball.
}
void Intake::IntakeBallOff() {
  IntakeMotor.Set(0);
  ConveyorMotor.Set(0);

  // Reverses the intake and conveyor at 1/4 speed.
}


void Intake::OnlyIntakeOn() {
  IntakeMotor.Set(.75);
}

void Intake::OnlyIntakeReverse() {
  IntakeMotor.Set(-.75);
}

void Intake::OnlyIntakeOff() {
  IntakeMotor.Set(0);
}

void Intake::ConveyBallOn() {
  ConveyorMotor.Set(.25);
}

void Intake::ConveyBallReverse() {
  ConveyorMotor.Set(-.25);
}

void Intake::ConveyBallOff() {
  ConveyorMotor.Set(0);
}



