/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Exhaust.h"

#include "Constants.h"

Exhaust::Exhaust()
    : Shoot(CANBusIds::VictorSpx::ShootMotor),
      Push(CANBusIds::VictorSpx::PushUpMotor) {
  SetSubsystem("Exhaust");
  Shoot.SetInverted(true);
  Push.SetInverted(true);
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

void Exhaust::ShootBallDown() {
  Shoot.Set(-1);
}

void Exhaust::ShootBallOff() {
  Shoot.Set(0);
}
