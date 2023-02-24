// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

#undef ENABLE_CLAMP_MOTORS

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
#ifdef ENABLE_CLAMP_MOTORS
  rev::CANSparkMax m_intakeClamp{MotorIds::SparkMax::INTAKE_MOTOR_CLAMP_ID,
                                 rev::CANSparkMax::MotorType::kBrushless};
#endif
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
