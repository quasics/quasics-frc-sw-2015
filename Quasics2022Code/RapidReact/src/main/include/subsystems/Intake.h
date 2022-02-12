// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Stop() {
    SetIntakeSpeed(0);
  }

  void SetIntakeSpeed(double intakeSpeed);

 private:
  ctre::phoenix::motorcontrol::can::VictorSPX m_floorPickupMotor{
      MotorIds::INTAKE_MOTOR_ID};
};
