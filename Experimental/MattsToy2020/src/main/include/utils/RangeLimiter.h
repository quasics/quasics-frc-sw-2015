/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Quasics Robotics and Matthew J. Healy                   */
/* All Rights Reserved.                                                       */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the BSD license file in the root directory of the   */
/* project repository.                                                        */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>

/**
 * Utility class to constrain input values to a fixed range (e.g., to that
 * provided by a standard joystick).
 */
template <typename ValueType>
class RangeLimiter {
 public:
  /** Constructor: will bind limits to +/- the parameter value. */
  constexpr RangeLimiter(ValueType absoluteLimit)
      : RangeLimiter(std::abs(absoluteLimit)) {
  }

  /**
   * Constructor: will bind limits to [lowValue, highValue].
   *
   * Note: if the caller swaps the low/high values, they will be re-ordered to
   * make sense.
   */
  constexpr RangeLimiter(ValueType lowValue, ValueType highValue)
      : lowerLimit(highValue >= lowValue ? lowValue : highValue),
        upperLimit(highValue >= lowValue ? highValue : lowValue) {
  }

  /**
   * Returns the specified value, constrained to fit in the specified min/max
   * range.
   */
  ValueType operator()(ValueType val) const {
    if (val < lowerLimit) {
      return lowerLimit;
    } else if (val > upperLimit) {
      return upperLimit;
    } else {
      return val;
    }
  }

 private:
  /// Lower end of the constraining range.
  ValueType lowerLimit;
  /// Upper end of the constraining range.
  ValueType upperLimit;
};
