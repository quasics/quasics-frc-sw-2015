// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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
     constexpr auto kP = 0.5; // changed from 1
     constexpr auto kI = 0.5;
     constexpr auto kD = 0;
  }
}