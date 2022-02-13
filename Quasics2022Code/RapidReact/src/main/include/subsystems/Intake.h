// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

/**
 * Floor intake subsystem, used to pick the balls from the floor (and then
 * transfer them to the conveyor).
 *
 * @see Conveyor
 */
class Intake : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  Intake();

  /**
   * Sets the speed of the intake to the specified percentage (-1.0 to +1.0).
   */
  void SetIntakeSpeed(double percentSpeed);

  /**
   * Convenience method to stop the intake.
   */
  void Stop() {
    SetIntakeSpeed(0);
  }

  // Standard functions for subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Data members.
 private:
  ctre::phoenix::motorcontrol::can::VictorSPX m_floorPickupMotor{
      MotorIds::VictorSPX::INTAKE_MOTOR_ID};
};
