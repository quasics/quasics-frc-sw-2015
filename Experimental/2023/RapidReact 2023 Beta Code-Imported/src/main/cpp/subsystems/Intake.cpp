// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {
  SetName("Intake");
}

// This method will be called once per scheduler run
void Intake::Periodic() {
}

void Intake::SetIntakeSpeed(double percentSpeed) {
  m_floorPickupMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      -percentSpeed);
}
