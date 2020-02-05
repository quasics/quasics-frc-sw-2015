/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Exhaust.h"

constexpr int PushUpMotor = 3;
constexpr int ShootMotor = 4;

Exhaust::Exhaust() : Shoot(ShootMotor), Push(PushUpMotor) {
}

// This method will be called once per scheduler run
void Exhaust::Periodic() {
}

void Exhaust::PushBallUp() {
  Push.Set(1);
}
void Exhaust::PushBallDown() {
  Push.Set(-1);
}
void Exhaust::PushBallOff() {
  Push.Set(0);
}
void Exhaust::ShootBallOn() {
  Shoot.Set(1);
}
void Exhaust::ShootBallOff() {
  Shoot.Set(0);
}