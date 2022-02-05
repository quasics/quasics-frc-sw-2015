// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {

}

// This method will be called once per scheduler run
void Shooter::Periodic() {
}

void Shooter::SetFlywheelSpeed(double flyWheelSpeed) {
    m_flyWheel.Set(flyWheelSpeed);
}
