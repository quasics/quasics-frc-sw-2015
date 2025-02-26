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

  /**
   * Constructor.
   * 
   * @param minVal  minimum (low) value for the deadband range
   * @param maxVal  maximum (high) value for the deadband range
   * @param deadVal value to be returned when a subsequent check indicates that
   *                the input is in the deadband (frequently 0, but not
   *                necessarily)
   */
  public DeadbandEnforcer(double minVal, double maxVal, double deadVal) {
    m_minVal = Math.min(minVal, maxVal);
    m_maxVal = Math.max(minVal, maxVal);
    m_deadVal = deadVal;
  }

  /**
   * Constructor. Will use 0 as the deadband value.
   * 
   * @param minVal minimum (low) value for the deadband range
   * @param maxVal maximum (high) value for the deadband range
   */
  public DeadbandEnforcer(double minVal, double maxVal) {
    this(minVal, maxVal, 0);
  }

  /**
   * Constructor. Will use 0 as the deadband value, and [-val,+val] as the
   * deadband range.
   * 
   * @param val value defining the deadband range (+/-)
   */
  public DeadbandEnforcer(double val) {
    this(-val, val);
  }

  /**
   * Evaluates the specified value in the context of the configured deadband
   * range.
   * 
   * @param val value being evaluated
   * @return val if it is outside the deadband, or the configured deadband value
   *         if val is within the deadband range
   */
  public double limit(double val) {
    if (val <= m_minVal || val >= m_maxVal) {
      return val;
    }
    return m_deadVal;
  }
}