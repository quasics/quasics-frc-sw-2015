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
 * Utility class, providing support for determining if a given value is within
 * an acceptable distance from a target (e.g., if a servo is nearly at a point
 * of full extension/retraction, etc.).
 */
template <typename T>
class WithinToleranceChecker {
 private:
  const T m_target;           ///< Target value
  const T m_acceptableDelta;  ///< Acceptable distance from the target (+/-)

 public:
  /**
   * Constructor.
   *
   * @param targetValue  the value we're trying to get close to
   * @param acceptableDelta  the delta from the target (+/-) that we're willing
   *                         to accept
   */
  WithinToleranceChecker(T targetValue, T acceptableDelta)
      : m_target(targetValue), m_acceptableDelta(acceptableDelta) {
  }

  /**
   * Functor used to check if a given value is within range of the target.
   *
   * @param value  value being evaluated
   * @return true iff the value is within the acceptable range
   */
  bool operator()(T value) const {
    return (std::abs(m_target - value) <= m_acceptableDelta);
  }
};