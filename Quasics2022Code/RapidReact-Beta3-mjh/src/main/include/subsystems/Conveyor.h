// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

/**
 * Conveyor subsystem, used to move the balls from the floor intake to where
 * they can be handled by the shooter.
 *
 * @see Intake
 * @see Shooter
 */
class Conveyor : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  Conveyor();

  /**
   * Sets the speed of the conveyor to the specified percentage (-1.0 to +1.0).
   */
  void SetConveyorSpeed(double percentSpeed);

  /**
   * Convenience method to stop the conveyor.
   */
  void Stop() {
    SetConveyorSpeed(0);
  }

  // Standard functions for subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Data members.
 private:
  ctre::phoenix::motorcontrol::can::VictorSPX m_conveyorMotor{
      MotorIds::VictorSPX::CONVEYOR_MOTOR_ID};
};
