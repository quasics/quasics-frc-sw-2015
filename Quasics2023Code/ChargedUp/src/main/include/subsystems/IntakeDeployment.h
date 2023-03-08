// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeDeployment : public frc2::SubsystemBase {
 public:
  IntakeDeployment();

  void SetMotorSpeed(double percentSpeed);

  void Stop();

  void ResetEncoders();

  double GetLeftPosition();

  double GetLeftVelocity();

  double GetRightPosition();

  double GetRightVelocity();

  void EnableBraking(bool value);

  bool IsIntakeDeployed(bool extend);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  rev::CANSparkMax m_leftDeploymentMotor{
      MotorIds::SparkMax::LEFT_INTAKE_DEPLOYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightDeploymentMotor{
      MotorIds::SparkMax::RIGHT_INTAKE_DEPLOYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};

  frc::MotorControllerGroup m_intakeDeployment{m_leftDeploymentMotor,
                                               m_rightDeploymentMotor};

  rev::SparkMaxRelativeEncoder m_leftDeploymentEncoder =
      m_leftDeploymentMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder m_rightDeploymentEncoder =
      m_rightDeploymentMotor.GetEncoder();

#endif

#ifdef ENABLE_INTAKE_LIMIT_SWITCH
  frc::DigitalInput m_leftExtendIntakeLimitSwitch{
      DigitalInput::INTAKE_EXTEND_LEFT_LIMIT_SWITCH_ID};
  frc::DigitalInput m_rightExtendIntakeLimitSwitch{
      DigitalInput::INTAKE_EXTEND_RIGHT_LIMIT_SWITCH_ID};
  frc::DigitalInput m_leftRetractIntakeLimitSwitch{
      DigitalInput::INTAKE_RETRACT_LEFT_LIMIT_SWITCH_ID};
  frc::DigitalInput m_rightRetractIntakeLimitSwitch{
      DigitalInput::INTAKE_RETRACT_RIGHT_LIMIT_SWITCH_ID};
#endif
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
