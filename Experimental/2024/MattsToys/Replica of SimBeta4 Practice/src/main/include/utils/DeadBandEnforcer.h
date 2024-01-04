#pragma once

#include <cmath>

//I believe this is simply a thing that ensures the robot accelerates at a constant rate and doesnt just
//jump from 0 to 100

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
