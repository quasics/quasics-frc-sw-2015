// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.Supplier;

/**
 * General interface for things that will manipulate motor speeds (e.g., as
 * specified by the driver
 * joysticks).
 *
 * <p>
 * Note: I expect that this interface will often just be implemented via a
 * lambda, or by using
 * convenience methods defined on the interface.
 */
public interface SpeedModifier {
  /**
   * Takes the specified input %age, and applies some type of modifier to it
   * (e.g., for "dead band"
   * handling, or speed caps, etc.).
   *
   * @param inputPercentage input %age of motor speed, to be adjusted
   * @return the modified value for the speed to be applied to a motor
   */
  double adjustSpeed(double inputPercentage);

  ////////////////////////////////////////////////////////////////////////
  // Convenience functions, to create commont types of SpeedControllers

  /**
   * Generates a speed modifier that implements a "deadband", where if lowValue <
   * inputPercentage <
   * highValue, then we'll treat it as "don't move"; otherwise, we'll just return
   * the input speed.
   *
   * <p>
   * This is meant to compensate for things like a joystick not being fully
   * calibrated, so that
   * it will appear to be reporting a non-zero value when the driver isn't
   * actually pushing the
   * stick in one direction or another.
   *
   * @param lowValue  the lower limit for the deadband (e.g., -0.05)
   * @param highValue the upper limit for the deadband (e.g., -0.05)
   * @return a SpeedModifier that handles deadband computations
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
      if (inputPercentage > sanityCheckedLowValue && inputPercentage < sanityCheckedHighValue) {
        return 0;
      }

      // OK, it's safe to use the input as the speed.
      return inputPercentage;
    };
  }

  /**
   * Generates a speed modifier that handles a "deadband", where if
   * Math.abs(input) <
   * Math.abs(deadband), then we'll treat it as "don't move"; otherwise, we'll
   * just return the input
   * speed.
   *
   * @param deadbandValue the value (+/-) for which an input speed should be
   *                      treated as effectively
   *                      0
   * @return a SpeedModifier that handles deadband computations
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  static SpeedModifier generateDeadbandSpeedModifier(double deadbandValue) {
    return generateDeadbandSpeedModifier(-Math.abs(deadbandValue), Math.abs(deadbandValue));
  }

  /**
   * Generates SpeedModifiers that apply a scaling factor to an input speed (e.g.,
   * to implement
   * "turtle mode", etc.).
   *
   * @param scalingFactor the scaling factor to be applied to an input speed
   */
  static SpeedModifier generateSpeedScaler(double scalingFactor) {
    return (double inputPercentage) -> inputPercentage * scalingFactor;
  }

  /**
   * Provides an absolute bounding on speeds (e.g., "don't let the speed get above
   * +85% (forward),
   * or -75% (reverse)").
   *
   * @param min the final minimum speed allowed for the robot (typically, >= -1.0)
   * @param max the final maximum speed allowed for the robot (typically), <=
   *            +1.0)
   * @see https://en.wikipedia.org/wiki/Speed_(1994_film)
   */
  static SpeedModifier generateSpeedBounder(double min, double max) {
    // Handle cases where folks accidentally swap the min and max values (e.g.,
    // min = +0.95 and highValue = -0.85). This way, we'll always use the
    // lesser value for the low end, etc.
    final double stableMin = Math.min(min, max);
    final double stableMax = Math.max(min, max);

    // Detect pathological cases, where the robot would never be allowed to stop,
    // because it's speed *couldn't* go to 0.
    if (stableMin > 0 || stableMax < 0) {
      // This would be a pathological case, where the robot is never allowed to stop,
      // because it's speed *couldn't* go to 0. I'm going to treat this as an error,
      // and just not provide any limits.
      System.err.println("**** Bullock/Reaves error detected: refusing to set speed bounds!");
      return (double inputPercentage) -> inputPercentage;
    }

    return (double inputPercentage) -> {
      if (inputPercentage > stableMax) {
        return stableMax;
      } else if (inputPercentage < stableMin) {
        return stableMin;
      } else
        return inputPercentage;
    };
  }

  /**
   * Provides an absolute bounding on speeds (e.g., "don't let the speed get above
   * 85% (forward or
   * backward)").
   *
   * @param absoluteLimit the final maximum % speed allowed for the robot
   *                      (positive or negative)
   * @see #generateSpeedBounder(double, double)
   */
  static SpeedModifier generateSpeedBounder(double absoluteLimit) {
    absoluteLimit = Math.abs(absoluteLimit);
    return generateSpeedBounder(-absoluteLimit, absoluteLimit);
  }

  /**
   * Generates a SpeedModifier that implements support for "normal/turtle/turbo"
   * mode decisions.
   *
   * @param normalScalingFactor scaling factor used when in "normal" mode (e.g.,
   *                            0.60)
   * @param turtleEnabled       supplies the signal to see if turtle mode is
   *                            active
   * @param turtleScalingFactor scaling factor used when in "turtle" mode (e.g.,
   *                            0.40)
   * @param turboEnabled        supplies the signal to see if turbo mode is active
   * @param turboScalingFactor  scaling factor used when in "normal" mode (e.g.,
   *                            0.85)
   */
  static SpeedModifier generateTurtleTurboSpeedModifier(
      final double normalScalingFactor,
      final Supplier<Boolean> turtleEnabled,
      final double turtleScalingFactor,
      final Supplier<Boolean> turboEnabled,
      final double turboScalingFactor) {
    final SpeedModifier turtleScaler = generateSpeedScaler(turtleScalingFactor);
    final SpeedModifier turboScaler = generateSpeedScaler(turboScalingFactor);
    final SpeedModifier normalScaler = generateSpeedScaler(normalScalingFactor);
    return (double inputPercentage) -> {
      if (turtleEnabled.get()) {
        return turtleScaler.adjustSpeed(inputPercentage);
      } else if (turboEnabled.get()) {
        return turboScaler.adjustSpeed(inputPercentage);
      } else {
        return normalScaler.adjustSpeed(inputPercentage);
      }
    };
  }

  /**
   * Generates a SpeedModifier that implements support for "normal/turtle/turbo"
   * mode decisions.
   *
   * @param normalScalingFactor    scaling factor used when in "normal" mode
   *                               (e.g., 0.60)
   * @param turtleEnabled          supplies the signal to see if turtle mode is
   *                               active
   * @param turtleScalingFactor    scaling factor used when in "turtle" mode
   *                               (e.g., 0.40)
   * @param turboEnabled           supplies the signal to see if turbo mode is
   *                               active
   * @param turboScalingFactor     scaling factor used when in "turbo" mode (e.g.,
   *                               0.85)
   * @param overdriveEnabled       supplies the signal to see if overdrive mode is
   *                               active
   * @param overdriveScalingFactor scaling factor used when in "overdrive" mode
   *                               (e.g., 1.00)
   */
  static SpeedModifier generateTurtleTurboOverdriveSpeedModifier(
      final double normalScalingFactor,
      final Supplier<Boolean> turtleEnabled,
      final double turtleScalingFactor,
      final Supplier<Boolean> turboEnabled,
      final double turboScalingFactor,
      final Supplier<Boolean> overdriveEnabled,
      final double overdriveScalingFactor) {
    final SpeedModifier turtleScaler = generateSpeedScaler(turtleScalingFactor);
    final SpeedModifier turboScaler = generateSpeedScaler(turboScalingFactor);
    final SpeedModifier overdriveScaler = generateSpeedScaler(overdriveScalingFactor);
    final SpeedModifier normalScaler = generateSpeedScaler(normalScalingFactor);
    return (double inputPercentage) -> {
      if (overdriveEnabled.get()) {
        return overdriveScaler.adjustSpeed(inputPercentage);
      } else if (turtleEnabled.get()) {
        return turtleScaler.adjustSpeed(inputPercentage);
      } else if (turboEnabled.get()) {
        return turboScaler.adjustSpeed(inputPercentage);
      } else {
        return normalScaler.adjustSpeed(inputPercentage);
      }
    };
  }

  /**
   * Generates a SpeedModifier that wraps the WPILib "slew rate limiter"
   * functionality, in order to
   * cap the acceleration rate (and smoothing out the speed curve).
   *
   * @param maximumSlewRate maximum change in the speed (as an absolute %age) per
   *                        second of the
   *                        robot (e.g., .5 means it would take 2 seconds to go to
   *                        full speed)
   * @see edu.wpi.first.math.filter.SlewRateLimiter
   */
  public static SpeedModifier generateSlewRateLimitModifier(double maximumSlewRate) {
    final SlewRateLimiter filter = new SlewRateLimiter(maximumSlewRate);
    return (double inputPercentage) -> {
      return filter.calculate(inputPercentage);
    };
  }
}
