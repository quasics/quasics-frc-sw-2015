// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

class Shooter : public frc2::SubsystemBase {
 public:
  static constexpr double POSITION_DELTA = 0.05;

 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //
  // Shooting control
 public:
  /// Sets speed for the shooting motor (-1.0 to +1.0).
  void SetSpeed(double speed);

  /// Stops the shooting motor.
  void Stop();

  //
  // Shooter angular control
 public:
  /** @return servo extension as a percentage of range (0.0 - 1.0) */
  double GetServoPosition();

  /**
   * @param pos   percent extension, expressed as [0.0 - 1.0]
   */
  void SetServoPosition(double pos);

  /** Increases shooting angle by POSITION_DELTA on the linear servo. */
  void IncrementPosition() {
    SetServoPosition(GetServoPosition() + POSITION_DELTA);
  }

  /** Decreases shooting angle by POSITION_DELTA on the linear servo. */
  void DecrementPosition() {
    SetServoPosition(GetServoPosition() - POSITION_DELTA);
  }

 private:
  /** Actual shooter: sending the ball out. */
  ctre::phoenix::motorcontrol::can::WPI_TalonFX shootingMotor;

  /** Experimental: servo to adjust the shooting angle. */
  frc::Servo positionServo;
};
