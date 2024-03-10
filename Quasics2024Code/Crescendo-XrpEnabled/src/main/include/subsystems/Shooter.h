// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

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

 private:
  // Data members.
 private:
  rev::CANSparkMax m_flyWheel;

  // TODO: (Matthew) If it's a follower, why do you need to talk to it?  (In
  // other words, we shouldn't need this 2nd object.)
  rev::CANSparkMax m_flyWheelTwo;
};
