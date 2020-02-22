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
    return (std::abs(m_target - value) <= m_acceptableDelta) ||
           (std::abs(value - m_target) <= m_acceptableDelta);
  }
};