// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SimulatedDriveBase.h"

constexpr auto kS = 1_V;
constexpr auto kV = 3 * (1_V * 1_s / 1_m);
constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 8.5;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::meter_t SIMULATED_TRACK_WIDTH_METERS = 0.381_m * 2;

SimulatedDriveBase::SimulatedDriveBase()
    : IDrivebase(SIMULATED_TRACK_WIDTH_METERS, kP, kI, kD, kS, kV, kA),
      m_odometry{frc::Rotation2d(), units::meter_t(0), units::meter_t(0)} {
}
