// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeDeployment : public frc2::SubsystemBase {
 public:
  IntakeDeployment();

  void SetMotorSpeed(double percentSpeed);

  void Stop();

  void EnableBraking(bool value);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_leftDeploymentMotor{
      MotorIds::SparkMax::LEFT_INTAKE_DEPLOYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightDeploymentMotor{
      MotorIds::SparkMax::RIGHT_INTAKE_DEPLYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};

  frc::MotorControllerGroup m_intakeDeployment{m_leftDeploymentMotor,
                                               m_rightDeploymentMotor};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
