package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Distance;

/**
 * Defines a simple interface to something running on a single motor.
 *
 * This is intended to be an example of an interface to some aspect of the robot, which would then
 * be implemented for specific hardware variants.
 */
public interface ISingleMotorThing {
  /**
   * Sets the speed of the motor.
   *
   * @param percent percent speed (a value in the range [-1.0, +1.0])
   */
  void setSpeed(double percent);

  /** Returns the motor speed. */
  double getSpeed();

  /** Returns the current position reading for the encoder on the "single motor thing". */
  Distance getPosition();

  /** Utility method to stop the motor. */
  default void stop() {
    setSpeed(0);
  }
}
