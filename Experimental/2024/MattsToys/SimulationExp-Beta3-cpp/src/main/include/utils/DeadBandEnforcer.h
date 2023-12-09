// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

class DeadBandEnforcer {
 private:
  const double m_lowerBand;
  const double m_upperBand;
  const double m_deadSetting;

 public:
  DeadBandEnforcer(double band) : DeadBandEnforcer(-band, band, 0.0) {
  }
  DeadBandEnforcer(double lowerBand, double upperBand)
      : DeadBandEnforcer(lowerBand, upperBand, 0.0) {
  }
  DeadBandEnforcer(double lowerBand, double upperBand, double deadSetting)
      : m_lowerBand(std::min(lowerBand, upperBand)),
        m_upperBand(std::max(lowerBand, upperBand)),
        m_deadSetting(deadSetting) {
  }
  double operator()(double value) const {
    if (value > m_lowerBand && value < m_upperBand) {
      return m_deadSetting;
    } else {
      return value;
    }
  }
};
