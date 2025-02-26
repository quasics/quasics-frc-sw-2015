// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Encapsulates "deadband" handling.
 *
 * Note: this does the same thing as MathUtil.applyDeadband, but allows us to
 * embed the limits in a (shareable) object.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Deadband">Deadband
 *      (Wikipedia)</a>
 */
public class DeadbandEnforcer {
  /** Defines the lower end of the deadband (inclusive). */
  final private double m_minVal;

  /** Defines the upper end of the deadband (inclusive). */
  final private double m_maxVal;

  /** Value to be returned if an input falls into the deadband. (Usually 0.) */
  final private double m_deadVal;

  public DeadbandEnforcer(double minVal, double maxVal, double deadVal) {
    m_minVal = Math.min(minVal, maxVal);
    m_maxVal = Math.max(minVal, maxVal);
    m_deadVal = deadVal;
  }

  public DeadbandEnforcer(double minVal, double maxVal) {
    this(minVal, maxVal, 0);
  }

  public DeadbandEnforcer(double val) {
    this(-val, val);
  }

  public double limit(double val) {
    if (val <= m_minVal || val >= m_maxVal) {
      return val;
    }
    return m_deadVal;
  }
}