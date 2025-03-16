// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Interface for a sensor that can be in a "triggered" state or not.
 * 
 * Context: the goal is to allow us to hide the logic for how to evaluate the
 * state of the switch, such as:
 * <ul>
 * <li>
 * A limit switch can indicate its "triggered" state returning true (if
 * configured "normally open") or false (if configured "normally closed").
 * </li>
 * <li>
 * An analog sensor/input might have a threshold value that indicates the sensor
 * has reached the "triggered" state (e.g., if a joystick is moved past some
 * position).
 * </li>
 * <li>
 * An input might be able to indicate multiple discrete states (e.g., a D-pad on
 * a controller), with each representing a distinct "triggered" state.
 * </li>
 * </ul>
 * 
 * For other options, see the WPILib docs for things like POVButton,
 * InternalButton, etc.
 * 
 * @see <a
 *      href=
 *      "https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/POVButton.html">POVButton</a>
 * @see <a
 *      href=
 *      "https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/InternalButton.html">InternalButton</a>
 */
public interface ITriggerSensor {
  /** @return true iff the sensor is in the "triggered" state */
  boolean isTriggered();

  /**
   * Create a sensor that is triggered when the given digital input returns the
   * specified value.
   * 
   * @param port         the port number of the digital input
   * @param triggerValue the "triggered" value of the digital input
   * @return a sensor that is triggered when the given digital input returns the
   *         specified value
   */
  static ITriggerSensor createForDigitalInput(int port, boolean triggerValue) {
    return new ITriggerSensor() {
      DigitalInput digitalInput = new DigitalInput(port);

      @Override
      public boolean isTriggered() {
        return digitalInput.get() == triggerValue;
      }
    };
  }
}
