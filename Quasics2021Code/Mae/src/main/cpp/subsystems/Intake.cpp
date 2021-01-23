/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"

#include "Constants.h"

using ctre::phoenix::motorcontrol::Brake;
using ctre::phoenix::motorcontrol::Coast;

Intake::Intake()
    : ballIntake(CANBusIds::VictorSpx::BallIntakeMotor),
      shoulder(CANBusIds::VictorSpx::ShoulderMotor) {
  SetSubsystem("Intake");
  SetShoulderToBrakeWhenNeutral();
}

void Intake::SetShoulderToBrakeWhenNeutral() {
  shoulder.SetNeutralMode(Brake);
}

void Intake::SetShoulderToCoastWhenNeutral() {
  shoulder.SetNeutralMode(Coast);
}

// This method will be called once per scheduler run
void Intake::Periodic() {
}

void Intake::TurnSuctionOn() {
  ballIntake.Set(.5);
}

void Intake::TurnSuctionOff() {
  ballIntake.Set(0);
}

void Intake::TurnSuctionOnReverse() {
  ballIntake.Set(-.5);
}

void Intake::RotateShoulderUp() {
  shoulder.Set(0.25);
}

void Intake::RotateShoulderDown() {
  shoulder.Set(-0.25);
}

void Intake::TurnShoulderOff() {
  shoulder.Set(0);
}
