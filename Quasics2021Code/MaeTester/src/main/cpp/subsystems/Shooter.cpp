// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include "Constants.h"

Shooter::Shooter() : shootingMotor(CANBusIds::VictorSPXIds::ShootingMotor) {
}

// This method will be called once per scheduler run
void Shooter::Periodic() {}

void Shooter::SetSpeed(double speed) {
  shootingMotor.Set(-speed);  // Yeet the ball
}

void Shooter::Stop() {
  shootingMotor.Set(0);
}