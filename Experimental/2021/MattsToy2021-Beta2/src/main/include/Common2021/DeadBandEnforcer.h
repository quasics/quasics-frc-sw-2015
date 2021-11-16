#pragma once

#include <algorithm>

/**
 * Utility class to provide a "dead band" around the center position on a
 * joystick, intended to handle cases where the stick isn't quite zeroed
 * when it's at rest.
 */
class DeadBandEnforcer {
 public:
  /**
   * Constructor.
   *
   * @param deadBandLimit  defines the +/- value for the dead band
   */
  DeadBandEnforcer(double deadBandLimit)
      : DeadBandEnforcer(deadBandLimit, -deadBandLimit) {}

  /**
   * Constructor.
   *
   * @param lowValue  defines the floor for the dead band (usually negative)
   * @param highValue  defines the ceiling for the dead band (usually positive)
   */
  DeadBandEnforcer(double lowValue, double highValue)
      : m_lowValue(std::min(lowValue, highValue)),
        m_highValue(std::max(lowValue, highValue)) {}

  /**
   * Applies the dead band to the specified value.
   *
   * @param value  the value to be evaluated against the dead band
   *
   * @return the provided value if it outside of the configured dead band;
   * otherwise, 0
   */
  double operator()(double value) const {
    if (value >= m_lowValue && value <= m_highValue)
      return 0;
    else
      return value;
  }

 private:
  double m_lowValue;   ///< Floor for the dead band
  double m_highValue;  ///< Ceiling for the dead band
};