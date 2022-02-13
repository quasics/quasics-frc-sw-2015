// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Conveyor.h"

Conveyor::Conveyor() {
  SetName("Conveyor");
}

void Conveyor::SetConveyorSpeed(double conveyorSpeed) {
  m_conveyorMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      conveyorSpeed);
}

// This method will be called once per scheduler run
void Conveyor::Periodic() {
}
