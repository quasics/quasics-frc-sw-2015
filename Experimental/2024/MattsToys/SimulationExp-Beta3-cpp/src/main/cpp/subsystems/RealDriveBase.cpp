// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDriveBase.h"

constexpr auto kS = 0.25829_V;
constexpr auto kV = 4.5623 * (1_V * 1_s / 1_m);
constexpr auto kA = 1.608 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 5.1527;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::meter_t TRACK_WIDTH_METERS_GLADYS = 0.559_m;

RealDriveBase::RealDriveBase()
    : IDrivebase(TRACK_WIDTH_METERS_GLADYS, kP, kI, kD, kS, kV, kA),
      m_odometry{m_trivialGyro->getRotation2d(),
                 m_leftTrivialEncoder->getPosition(),
                 m_rightTrivialEncoder->getPosition()} {
  SetName("RealDriveBase");
}
