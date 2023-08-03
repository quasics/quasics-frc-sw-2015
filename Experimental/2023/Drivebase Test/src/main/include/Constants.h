// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <units/acceleration.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#define USING_GLADYS_TRAJECTORY_CONSTANTS
#undef USING_SALLY_TRAJECTORY_CONSTANTS
#undef USING_MAE_TRAJECTORY_CONSTANTS

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

  // Intake deployment controls (same for both intake options)
  constexpr int RIGHT_INTAKE_DEPLOYMENT_MOTOR_ID = 5;
  constexpr int LEFT_INTAKE_DEPLOYMENT_MOTOR_ID = 6;

  // Constants for Intake option #1
  constexpr int INTAKE_MOTOR_CLAMP_ID = 7;

  // Constants for Intake option #2
  constexpr int INTAKE_MOTOR_ROLLER_ID = 7;
}  // namespace SparkMax

namespace VictorSPX {
  constexpr int GAME_PIECE_FLIPPER_ID = 1;
}
}  // namespace MotorIds

namespace ThroughBore {
constexpr int A_CHANNEL = 0;
constexpr int B_CHANNEL = 1;
}  // namespace ThroughBore

namespace OperatorInterface {
// Joystick slot used for "driver" controller (normally Logitech hardware).
constexpr int DRIVER_JOYSTICK = 0;

// Joystick slot used for "operator" controller (normally Xbox hardware).
constexpr int OPERATOR_JOYSTICK = 1;

using RateLimit = units::unit_t<
    units::compound_unit<units::scalar, units::inverse<units::seconds>>>;

/** Limit on robot acceleration. */
constexpr RateLimit DRIVER_JOYSTICK_RATE_LIMIT = 1 / 1_s;

namespace LogitechGamePad {
  // Axes - Used with the "getRawAxis()" function to access the data for the
  // individual sticks on the controller (e.g., for "tank drive" coding).
  //
  // Note: these values assume that the switch on the bottom of the Logitech
  // controller is in the "D" position.
  //
  // If the switch is in the "D" position, the controller will enumerate as a
  // Logitech Dualshock controller, the right joystick X/Y axes are 2and 3,
  // respectively, and the left and right triggers show up as *buttons* 7 and 8.
  //
  // If the switch is in the "X" position, it will enumerate as a Logitech
  // Gamepad F310. In this mode, the right joystick X/Y axes are 4 and 5,
  // respectively, and the left and right triggers on the front enumerate as
  // joysticks 2 and 3.
  constexpr int LEFT_X_AXIS = 0;
  constexpr int LEFT_Y_AXIS = 1;
  constexpr int RIGHT_X_AXIS = 2;
  constexpr int RIGHT_Y_AXIS = 3;

  // Buttons
  constexpr int A_BUTTON = 2;  // Labeled "2" on some controllers
  constexpr int B_BUTTON = 3;  // Labeled "3" on some controllers
  constexpr int X_BUTTON = 1;  // Labeled "1" on some controllers
  constexpr int Y_BUTTON = 4;  // Labeled "4" on some controllers
  constexpr int LEFTSHOULDER = 5;
  constexpr int RIGHTSHOULDER = 6;
  constexpr int LEFT_TRIGGER = 7;
  constexpr int RIGHT_TRIGGER = 8;
  constexpr int BACK_BUTTON = 9;
  constexpr int START_BUTTON = 10;
  constexpr int LEFT_STICK_PRESS = 11;
  constexpr int RIGHT_STICK_PRESS = 12;
}  // namespace LogitechGamePad
}  // namespace OperatorInterface

namespace SensorIds {
constexpr int PIDGEON_CAN_ID = 1;
}


//Beggining The Trajectory Tutorial Stuff

namespace PathWeaverConstants {
#ifdef USING_MAE_TRAJECTORY_CONSTANTS
constexpr auto kS = 0.13895_V;
constexpr auto kV = 1.3143 * (1_V * 1_s / 1_m);
constexpr auto kA = 0.1935 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 0.001379;
constexpr double kI = 0;
constexpr double kD = 0;
#endif
#ifdef USING_GLADYS_TRAJECTORY_CONSTANTS
constexpr auto kS = 0.25829_V;
constexpr auto kV = 4.5623 * (1_V * 1_s / 1_m);
constexpr auto kA = 1.608 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 5.1527;
constexpr double kI = 0;
constexpr double kD = 0;
#endif
#ifdef USING_SALLY_TRAJECTORY_CONSTANTS
constexpr auto kS = 0.19529_V;
constexpr auto kV = 2.2329 * (1_V * 1_s / 1_m);
constexpr auto kA = 0.36638 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 0.29613;
constexpr double kI = 0;
constexpr double kD = 0;
#endif
}  // namespace PathWeaverConstants

constexpr units::length::meter_t TRACK_WIDTH_METERS_GLADYS = 0.559_m;
const frc::DifferentialDriveKinematics kDriveKinematics(TRACK_WIDTH_METERS_GLADYS);

constexpr auto kMaxSpeed = 1_mps;
constexpr auto kMaxAcceleration = 0.5_mps_sq;

constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

