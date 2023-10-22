// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

/**
 * TODO: Add comments describing the class as a whole.
 */
// CODE_REVIEW(ethan): This class should have a comment block (above) describing
// what it is/does.
class IntakeDeployment : public frc2::SubsystemBase {
 public:
  enum class LimitSwitch { Extended, Retracted };

  IntakeDeployment();

  void SetMotorSpeed(double percentSpeed);

  void Stop();

  void ResetEncoders();

  double GetLeftPosition();

  double GetLeftVelocity();

  double GetRightPosition();

  double GetRightVelocity();

  void EnableBraking(bool value);

  /**
   * Returns true if the limit switches are enabled on the intake, and if the
   * indicated limit switch is being triggered (on either side, if appropriate).
   */
  bool IsIntakeDeployed(LimitSwitch limitSwitch);

  // bool IsIntakeDeployed();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  ///////////////////////////////////////////////////////////////
  // Data members (motors, encoders, digital inputs, etc.)
 private:
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  // Motors that control the left/right side of the intake deployment hardware.
  rev::CANSparkMax m_leftDeploymentMotor{
      MotorIds::SparkMax::LEFT_INTAKE_DEPLOYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightDeploymentMotor{
      MotorIds::SparkMax::RIGHT_INTAKE_DEPLOYMENT_MOTOR_ID,
      rev::CANSparkMax::MotorType::kBrushless};

  // Encoders, to keep track of where the relative motion of the deployment
  // mechanism.
  rev::SparkMaxRelativeEncoder m_leftDeploymentEncoder =
      m_leftDeploymentMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightDeploymentEncoder =
      m_rightDeploymentMotor.GetEncoder();

  // Motor controller group, binding the left/right side motors into a single
  // pair.
  //
  // CODE_REVIEW(ethan): Is there a reason why we're using a controller group
  // (which allows motor speeds to be set independently), rather than just
  // having one of the motors configured as the "follower" of the other, so that
  // they automatically run at the same speeds all the time?
  frc::MotorControllerGroup m_intakeDeployment{m_leftDeploymentMotor,
                                               m_rightDeploymentMotor};
#endif

  // Limit switches for the intake deployment hardware.
  frc::DigitalInput m_leftExtendIntakeLimitSwitch{
      DigitalInput::INTAKE_EXTEND_LEFT_LIMIT_SWITCH_ID};
#ifdef ENABLE_EXPANDED_INTAKE_LIMIT_SWITCHES
  frc::DigitalInput m_rightExtendIntakeLimitSwitch{
      DigitalInput::INTAKE_EXTEND_RIGHT_LIMIT_SWITCH_ID};
  frc::DigitalInput m_leftRetractIntakeLimitSwitch{
      DigitalInput::INTAKE_RETRACT_LEFT_LIMIT_SWITCH_ID};
  frc::DigitalInput m_rightRetractIntakeLimitSwitch{
      DigitalInput::INTAKE_RETRACT_RIGHT_LIMIT_SWITCH_ID};
#endif
};
