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
