// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Defines a simple interface to something running on a single motor.
 *
 * This is intended to be an example of an interface to some aspect of the
 * robot, which would then
 * be implemented for specific hardware variants.
 */
public interface ISingleMotorThing {
  /**
   * Casts this object to a Subsystem. (Since anything implementing it should be
   * one.)
   */
  default Subsystem asSubsystem() {
    return (Subsystem) this;
  }

  /**
   * Casts this object to a Sendable. (Since anything implementing it should be
   * deriving from SubsystemBase, which is one.)
   */
  default Sendable asSendable() {
    return (Sendable) this;
  }

  /**
   * Sets the speed of the motor.
   *
   * @param percent percent speed (a value in the range [-1.0, +1.0])
   */
  void setSpeed(double percent);

  /** Returns the motor speed. */
  double getSpeed();

  /**
   * Returns the current position reading for the encoder on the "single motor
   * thing".
   */
  Distance getPosition();

  /** Utility method to stop the motor. */
  default void stop() {
    setSpeed(0);
  }
}
