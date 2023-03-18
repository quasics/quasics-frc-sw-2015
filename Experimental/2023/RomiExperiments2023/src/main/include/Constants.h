// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
  constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants


namespace DriveConstants {
  constexpr double kCountsPerRevolution = 1440.0;
  constexpr double kWheelDiameterInch = 2.75;
}  // namespace DriveConstants

namespace SelfBalancingConstants{
  namespace PID{
     constexpr auto ks = 0;
     constexpr auto kP = 0.1; // changed from 1
     constexpr auto kI = 0.0;
     constexpr auto kD = 0.0;
  }
}

namespace PhotonVisionConstants {
namespace CameraAndTargetValues {
  constexpr units::meter_t CAMERA_HEIGHT = 4_in;   // 19.25_in on real bot
  constexpr units::meter_t TARGET_HEIGHT = 0_in;  // 14.25on real field
  constexpr units::radian_t CAMERA_PITCH = 0_rad;
  const units::meter_t GOAL_RANGE_METERS = 2_ft;  // 3ft on real field
}  // namespace CameraAndTargetValues

namespace LinearPID {
  const double kP = 0.2;
  const double kI = 0.05;
  const double kD = 0.0;
}  // namespace LinearPID

namespace AngularPID {
  const double kP = 0.01;
  const double kI = 0.0;
  const double kD = 0.0;
}  // namespace AngularPID

} 