// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
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
  enum class LimitSwitch { Extended, Retracted };

  IntakeDeployment();

  void SetMotorSpeed(double percentSpeed);

  void Stop();

  void ResetEncoders();

  double GetPosition();

  double GetVelocity();

  double GetRevolutions();

  bool ExtendedByRevolutions();

  bool RetractedByRevolutions();

  void EnableBraking(bool value);

  bool IsIntakeDeployed();

  bool IsIntakeRetracted();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  ///////////////////////////////////////////////////////////////
  // Data members (motors, encoders, digital inputs, etc.)
 private:
  rev::CANSparkMax m_intakeDeployment{
      MotorIds::SparkMax::INTAKE_DEPLOYMENT_MOTOR,
      rev::CANSparkMax::MotorType::kBrushless};
  // Encoders, to keep track of where the relative motion of the deployment
  // mechanism.
  rev::SparkRelativeEncoder m_DeploymentEncoder = m_intakeDeployment.GetEncoder(
      rev::SparkRelativeEncoder::Type::kHallSensor);

  frc::DigitalInput m_ExtendIntakeLimitSwitch{
      DigitalInput::INTAKE_EXTEND_LIMIT_SWITCH_ID};
  frc::DigitalInput m_RetractIntakeLimitSwitch{
      DigitalInput::INTAKE_RETRACT_LIMIT_SWITCH_ID};
};
