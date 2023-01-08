// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * General interface for things that will manipulate motor speeds (e.g., as
 * specified by the driver joysticks).
 * 
 * Note: I expect that this interface will often just be implemented via a
 * lambda, or by using convenience methods defined on the interface.
 */
public interface SpeedModifier {
  /**
   * Takes the specified input %age, and applies some type of modifier to it
   * (e.g., for "dead band" handling, or speed caps, etc.).
   * 
   * @param inputPercentage input %age of motor speed, to be adjusted
   * @return the modified value for the speed to be applied to a motor
   */
  double adjustSpeed(double inputPercentage);

  ////////////////////////////////////////////////////////////////////////
  // Convenience functions, to create commont types of SpeedControllers

  /**
   * Generates a speed modifier that implements a "deadband", where if
   * lowValue < inputPercentage < highValue, then we'll treat it as "don't move";
   * otherwise, we'll just return the input speed.
   * 
   * This is meant to compensate for things like a joystick not being fully
   * calibrated, so that it will appear to be reporting a non-zero value when the
   * driver isn't actually pushing the stick in one direction or another.
   * 
   * @param lowValue  the lower limit for the deadband (e.g., -0.05)
   * @param highValue the upper limit for the deadband (e.g., -0.05)
   * @return a SpeedModifier that handles deadband computations
   * 
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  static SpeedModifier generateDeadbandSpeedModifier(double lowValue, double highValue) {
    // Handle cases where folks accidentally swap the low and high range (e.g.,
    // lowValue = +0.08 and highValue = -0.05). This way, we'll always use the
    // lesser value for the low end, etc.
    final double sanityCheckedLowValue = (lowValue < highValue ? lowValue : highValue);
    final double sanityCheckedHighValue = (highValue > lowValue ? highValue : lowValue);

    return (double inputPercentage) -> {
      // Does the input percentage fall into the deadband?
      if (inputPercentage > sanityCheckedLowValue
          && inputPercentage < sanityCheckedHighValue) {
        return 0;
      }

      // OK, it's safe to use the input as the speed.
      return inputPercentage;
    };
  }

  /**
   * Generates a speed modifier that handles a "deadband", where if
   * Math.abs(input) < Math.abs(deadband), then we'll treat it as "don't move";
   * otherwise, we'll just return the input speed.
   * 
   * @param deadbandValue the value (+/-) for which an input speed should be
   *                      treated as effectively 0
   * @return a SpeedModifier that handles deadband computations
   * 
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  static SpeedModifier generateDeadbandSpeedModifier(double deadbandValue) {
    return generateDeadbandSpeedModifier(-Math.abs(deadbandValue), Math.abs(deadbandValue));
  }
}
