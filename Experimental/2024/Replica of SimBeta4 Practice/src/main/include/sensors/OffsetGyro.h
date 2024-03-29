#pragma once

#include <iostream>

#include "sensors/IGyro.h"

//Come Back To THIS!!!! I dont understand why this is necessarily needed

class OffsetGyro : public IGyro {
 private:
  IGyro& m_srcGyro;
  units::degree_t m_calibrationOffset{0};

 public:

  OffsetGyro(IGyro& srcGyro) : m_srcGyro(srcGyro) {
  }

  virtual ~OffsetGyro() = default;

  void calibrate() override {
    std::cerr << "*** OffsetGyro: null calibration\n";
  }

  angle_t getAngle() override {
    return m_srcGyro.getAngle() - m_calibrationOffset;
  }

  rate_t getRate() override {
    return m_srcGyro.getRate();
  }

  frc::Rotation2d getRotation2d() override {
    return units::degree_t{-getAngle()};
  }

  void reset() override {
    m_calibrationOffset = m_srcGyro.getAngle();
  }
};