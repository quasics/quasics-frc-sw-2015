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
 * Utility class to encapsulate scaling an input value (e.g., as provided by a
 * joystick) by a fixed amount.
 */
template <typename ValueType>
class ValueScaler {
 public:
  /** Constructor. */
  constexpr ValueScaler(ValueType scalingFactor)
      : scalingFactor(scalingFactor) {
  }

  /** Applies the scaling factor to the parameter value. */
  ValueType operator()(ValueType val) const {
    return val * scalingFactor;
  }

 private:
  ValueType scalingFactor;
};
