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

template <typename T>
class WithinToleranceChecker {
 private:
  const T m_target;
  const T m_acceptableDelta;

 public:
  WithinToleranceChecker(T targetValue, T acceptableDelta)
      : m_target(targetValue), m_acceptableDelta(acceptableDelta) {
  }
  bool operator()(T value) const {
    return (std::abs(m_target - value) <= m_acceptableDelta);
  }
};