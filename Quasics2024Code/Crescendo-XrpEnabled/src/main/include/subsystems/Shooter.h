// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

/**
 * Cargo (ball) shooting subsystem, used to deliver cargo to the hub.
 */
class Shooter : public frc2::SubsystemBase {
 public:
  static constexpr double POSITION_DELTA = 0.05;

 public:
  Shooter();

  /**
   * Sets the speed of the shooter's flywheel to the specified percentage (-1.0
   * to +1.0).
   */
  void SetFlywheelSpeed(double percentSpeed);

  /** Convenience method to stop the shooter. */
  void Stop() {
    SetFlywheelSpeed(0);
  };

  void ExtendLinearActuators();

  void RetractLinearActuators();

  bool IsFullyExtended();

  bool IsFullyRetracted();

  // Standard functions for subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Data members.
 private:
  rev::CANSparkMax m_flyWheel{
      MotorIds::SparkMax::SHOOTER_FLYWHEEL_MOTOR_LEADER_ID,
      rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_flyWheelTwo{
      MotorIds::SparkMax::SHOOTER_FLYWHEEL_MOTOR_FOLLOWER_ID,
      rev::CANSparkMax::MotorType::kBrushless};

  frc::Servo m_leftPositionServo{PWMPorts::LEFT_SERVO};

  frc::Servo m_rightPositionServo{PWMPorts::RIGHT_SERVO};
};
