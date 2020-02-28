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
 * Utility class to provide a "dead band" in an input range, where values that
 * fall in that range will be converted to 0 (e.g., to compensate for slight
 * miscalibration of a joystick).
 */
template <typename ValueType>
class DeadBandLimiter {
 public:
  /** Constructor: will configure a dead band between +/- the parameter value.
   */
  constexpr DeadBandLimiter(ValueType absoluteLimit)
      : DeadBandLimiter(-absoluteLimit, absoluteLimit) {
  }

  /**
   * Constructor: will bind limits to [lowValue, highValue].
   *
   * Note: if the caller swaps the low/high values, they will be re-ordered to
   * make sense.
   */
  constexpr DeadBandLimiter(ValueType lowValue, ValueType highValue)
      : lowerLimit(highValue >= lowValue ? lowValue : highValue),
        upperLimit(highValue >= lowValue ? highValue : lowValue) {
  }

  /**
   * Returns the specified value, after taking into account the configured dead
   * band.
   */
  ValueType operator()(ValueType val) const {
    if (val >= lowerLimit && val <= upperLimit) {
      return 0;
    } else {
      return val;
    }
  }

 private:
  /// Lower end of the dead band.
  ValueType lowerLimit;
  /// Upper end of the dead band.
  ValueType upperLimit;
};
