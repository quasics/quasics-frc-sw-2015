// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>

#include "sensors/IGyro.h"

/**
 * Wrapper around an IGyro, allowing us to reset it "locally", without affecting
 * the original gyro's data.
 */
class OffsetGyro : public IGyro {
 private:
  IGyro& m_srcGyro;
  units::degree_t m_calibrationOffset{0};

 public:
  /**
   * Constructs an OffsetGyro, wrapped around a source Gyro.
   *
   * @param sourceGyro the Gyro used to obtain actual readings
   */
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
