// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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



namespace RobotPhysics {
/** Drive base gear ratio used for Mae (2020/2021 robot). */
constexpr double DRIVEBASE_GEAR_RATIO_MAE = 10.71;
/** Drive base gear ratio used for Sally (2022 robot/development drive base). */
constexpr double DRIVEBASE_GEAR_RATIO_SALLY = 8.45;
/** Drive base gear ratio used for Gladys (2023 robot). */
constexpr double DRIVEBASE_GEAR_RATIO_GLADYS = 8.45;

/**
 * Drive base gear ratio on the currently-targeted robot.
 *
 * Used to isolate the gear ratio for the robot we're actually building for,
 * from the various "known values" for each of the possible robots.
 */
constexpr double DRIVEBASE_GEAR_RATIO = DRIVEBASE_GEAR_RATIO_GLADYS;

/** Wheel diameter used with all of our 2020-2023 robot drive bases. */
constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;

/** Track width for Gladys. */
// TODO(matthew): Confirm that this is correct.
constexpr units::length::inch_t TRACK_WIDTH_INCHES_GLADYS = 22.0_in;
}  // namespace RobotPhysics

namespace MotorIds {
namespace SparkMax {
  constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
  constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
  constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
  constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
}
}

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
