// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

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
  // Bug(Josh): The "6" here needs to be replaced by the
  // "MotorIds.INTAKE_MOTOR_ID" constant.  (Define all of our IDs in one place,
  // so that have a single place to update mappings.)
  ctre::phoenix::motorcontrol::can::VictorSPX m_floorPickupMotor{6};
};
