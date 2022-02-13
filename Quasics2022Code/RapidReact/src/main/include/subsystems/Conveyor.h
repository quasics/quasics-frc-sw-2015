// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Conveyor : public frc2::SubsystemBase {
 public:
  Conveyor();

  void SetConveyorSpeed(double conveyorSpeed);

  void Stop() {
    SetConveyorSpeed(0);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ctre::phoenix::motorcontrol::can::VictorSPX m_conveyorMotor{
      MotorIds::VictorSPX::CONVEYOR_MOTOR_ID};
};
