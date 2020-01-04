/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

namespace DriveBaseConstants {
constexpr double kStandardPowerScalingFactor = 0.45;
constexpr double kTurboPowerScalingFactor = 0.45;
}  // namespace DriveBaseConstants

namespace OIConstants {
constexpr double kDriveControllerDeadBandSize = 0.015;
constexpr int kDriverControllerPort = 1;
}  // namespace OIConstants
