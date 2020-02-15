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

template <typename ValueType>
class RangeLimiter {
 public:
  constexpr RangeLimiter(ValueType absoluteLimit)
      : RangeLimiter(std::abs(absoluteLimit)) {
  }

  constexpr RangeLimiter(ValueType lowValue, ValueType highValue)
      : lowerLimit(highValue >= lowValue ? lowValue : highValue),
        upperLimit(highValue >= lowValue ? highValue : lowValue) {
  }

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
  ValueType lowerLimit;
  ValueType upperLimit;
};
