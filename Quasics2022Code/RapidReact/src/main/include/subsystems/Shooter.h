// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Stop() {
    SetFlywheelSpeed(0);
  };

  void SetFlywheelSpeed(double flyWheelSpeed);

 private:
  rev::CANSparkMax m_flyWheel{MotorIds::SparkMax::SHOOTER_FLYWHEEL_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
};
