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
