// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeClamp : public frc2::SubsystemBase {
 public:
  IntakeClamp();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetIntakeClampSpeed(double percentSpeed);

  void Stop();

  void EnableBraking(bool value);

  bool IsIntakeClampDeployed();

 private:
  rev::CANSparkMax m_intakeClamp{MotorIds::SparkMax::INTAKE_MOTOR_CLAMP_ID,
                                 rev::CANSparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
