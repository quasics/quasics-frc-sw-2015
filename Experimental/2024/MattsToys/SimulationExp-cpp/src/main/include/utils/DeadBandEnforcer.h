// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

// clang-format off
/**
 * This is a utility class to help provide a reusable implementation of
 * "deadband" handling to code.
 *
 * A "deadband" is generally used when you've got a range of values that can be
 * provided by some input, and there's a span <em>inside</em> that range that
 * you want to treat as being "effectively zero".
 *
 * A very common example of this sort of thing happens with gamepads.  When you
 * let go of the joystick, it often won't go back to precisely the "0" value,
 * but will instead wind up at a position that is slightly above/below that
 * point (e.g., +0.00832, or -0.01044).  If this value is passed directly on to
 * a motor or some other actuator as a "speed value", then the motor winds up
 * trying to spin at a very low voltage (which may or may not translate into
 * <em>actual</em> motion, depending on the mechanism).  Defining a deadband
 * will let you ensure that values that are "effectively zero" for your purposes
 * are actually <em>treated</em> as zero when taken as inputs for some kind of
 * control.
 *
 * Sample usage:
 * <code>
 *    DeadBandEnforcer deadband{0.01};  // Sets up [-0.01 to +0.01] as the "deadband"
 *    . . . .
 *    // If the left joystick is in the deadband range, then the reading will be
 *    // converted to 0; otherwise, it'll be whatever position the joystick is at.
 *    double joystickReading = deadband(m_driverController.GetLeftY());
 * </code>
 *
 * @see https://en.wikipedia.org/wiki/Deadband
 */
// clang-format on
class DeadBandEnforcer {
 private:
  /** Low end of the deadband. */
  const double m_lowerBand;
  /** High end of the deadband. */
  const double m_upperBand;
  /**
   * Value to return for inputs within the deadband.  (This will often be 0.)
   */
  const double m_deadSetting;

 public:
  /**
   * Constructor, setting up a deadband from "-band" to "+band", and using 0 as
   * the output for readings within that zone.
   */
  DeadBandEnforcer(double band) : DeadBandEnforcer(-band, band, 0.0) {
  }

  /**
   * Constructor, setting up a deadband from "lowerBand" to "upperBand", and
   * using 0 as the output for readings within that zone.
   *
   * @note If lowerBand > upperBand, then these values will be automatically
   * swapped when the deadband is defined (e.g., if lowerBand is +0.02 and
   * upperBand is -0.01, then the deadband range will be set up as [-0.01,
   * +0.02]).
   */
  DeadBandEnforcer(double lowerBand, double upperBand)
      : DeadBandEnforcer(lowerBand, upperBand, 0.0) {
  }

  /**
   * Constructor, setting up a deadband from "lowerBand" to "upperBand", and
   * return "deadSetting" as the output for readings within that zone.
   *
   * @note If lowerBand > upperBand, then these values will be automatically
   * swapped when the deadband is defined (e.g., if lowerBand is +0.02 and
   * upperBand is -0.01, then the deadband range will be set up as [-0.01,
   * +0.02]).
   */
  DeadBandEnforcer(double lowerBand, double upperBand, double deadSetting)
      : m_lowerBand(std::min(lowerBand, upperBand)),
        m_upperBand(std::max(lowerBand, upperBand)),
        m_deadSetting(deadSetting) {
  }

  /**
   * Applies the deadband to the input value, and returns a "normalized" version
   * of the input.
   *
   * @note This operator is the "function call operator" in C++, and allows a
   * DeadBandEnforcer variable name to be used as if it is the name of a
   * function.  (See the code example, above.)
   *
   * @return value if the input value is outside of the deadband range, or
   * m_deadSetting if it is inside the deadband.
   */
  double operator()(double value) const {
    if (value > m_lowerBand && value < m_upperBand) {
      return m_deadSetting;
    } else {
      return value;
    }
  }
};
