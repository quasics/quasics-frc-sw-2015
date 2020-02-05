/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"

constexpr int BallIntakeMotor = 1;
constexpr int ShoulderMotor = 2;

Intake::Intake()
  : BallIntake(BallIntakeMotor),
    Shoulder(ShoulderMotor){}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::TurnSuctionOn () {
    BallIntake.Set(1);
}
void Intake::TurnSuctionOff () {
    BallIntake.Set(0);
}
void Intake::TurnSuctionOnReverse () {
    BallIntake.Set(-1);
}
void Intake::RotateShoulderUp() {
    Shoulder.Set(1);
}
void Intake::RotateShoulderDown() {
    Shoulder.Set(-1);
}
void Intake::TurnShoulderOff() {
    Shoulder.Set(0);
}
