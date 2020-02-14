#pragma once

#include <cmath>

template <typename ValueType>
class ValueScaler {
 public:
  constexpr ValueScaler(ValueType scalingFactor)
      : scalingFactor(scalingFactor) {
  }

  ValueType operator()(ValueType val) const {
    return val * scalingFactor;
  }

 private:
  ValueType scalingFactor;
};
