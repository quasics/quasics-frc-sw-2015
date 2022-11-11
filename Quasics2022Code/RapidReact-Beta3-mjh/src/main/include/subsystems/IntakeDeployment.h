// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeDeployment : public frc2::SubsystemBase {
 public:
  IntakeDeployment();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void SetMotorSpeed(double);
  void SetBrakingMode(bool brake);

  bool IsIntakeDeployed();

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix::motorcontrol::can::VictorSPX m_IntakeDeploymentMotor{
      MotorIds::VictorSPX::INTAKE_DEPLOYMENT_MOTOR};

  frc::DigitalInput intakeLimitSwitch{DigitalInput::INTAKE_LIMIT_SWITCH_ID};
};
