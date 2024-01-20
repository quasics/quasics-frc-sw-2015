// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>

#include "sensors/IGyro.h"

// clang-format off
/**
 * Wrapper around an IGyro, allowing us to reset it "locally", without affecting
 * the original gyro's data.
 *
 * I foresee this as being useful under at least two different circumstances:
 * <ol>
 *   <li>
 *     When we're wrapping a multi-axis gyro/ALU inside of the single-axis "IGyro"
 *     interface, and want to be able to support resetting data for that axis under
 *     some circumstances, without impacting the others.
 *   </li>
 *   <li>
 *     When we're performing some temporally-scoped set of operations (e.g., while a
 *     command is running) and want to have an easy reference point (i.e., 0)
 *     without affecting the overall use of a gyro/ALU. For example, the gyro might
 *     be used on a continuous basis to maintain data for odometry/pose estimation,
 *     and thus resetting the actual gyro would "break" that processing. However, we
 *     might want to simplify the handling of a command like "turn N degrees" by
 *     allowing the code to use 0 as a reference point (and this class would be able
 *     to support that).
 *   </li>
 * </ol>
 */
// clang-format on
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

  /** (Trivial) destructor. */
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
