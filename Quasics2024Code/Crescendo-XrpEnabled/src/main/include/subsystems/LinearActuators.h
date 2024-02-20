// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

/**
 * Cargo (ball) shooting subsystem, used to deliver cargo to the hub.
 *
 * Note that the servos are assumed to be an AndyMark Linear Servo Actuator
 * L16-R 140 mm Stroke 35:1 6v, and need to be configured appropriately.
 */
class LinearActuators : public frc2::SubsystemBase {
 public:
  static constexpr double POSITION_DELTA = 0.05;

 public:
  LinearActuators();

  void ExtendLinearActuators();

  void RetractLinearActuators();

  // TODO: (Matthew) This isn't implemented yet.  If you try to use it, you'll
  // get a linker error.
  bool IsFullyExtended();

  // TODO: (Matthew) This isn't implemented yet.  If you try to use it, you'll
  // get a linker error.
  bool IsFullyRetracted();

 private:
  /** Configures a Servo object to be used with an AndyMark L16 actuator. */
  static void ConfigureAndyMarkL16(frc::Servo& servo);

  // Data members.
 private:
  frc::Servo m_leftPositionServo;
  frc::Servo m_rightPositionServo;
};
