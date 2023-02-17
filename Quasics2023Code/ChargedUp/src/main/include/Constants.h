// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;

constexpr double DRIVEBASE_GEAR_RATIO = 8.45;  // gear ratio for Sally is 8.45

namespace OperatorConstants {

/** Joystick port for the driver's controller. */
constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace AutonomousTeamAndStationPositions {
constexpr auto Blue1 = "Blue 1";
constexpr auto Blue2 = "Blue 2";
constexpr auto Blue3 = "Blue 3";
constexpr auto Red1 = "Red 1";
constexpr auto Red2 = "Red 2";
constexpr auto Red3 = "Red 3";

}  // namespace AutonomousTeamAndStationPositions

namespace AutonomousSelectedOperation {
const auto DoNothing = "Do nothing";
const auto GTFO = "GTFO";
const auto GTFODock = "GTFO and Dock";
const auto moveToDefense = "Move to defense";
}  // namespace AutonomousSelectedOperation

namespace SelfBalancingConstants {
namespace PID {
constexpr auto ks = 0;
constexpr auto kP = 0.025;
constexpr auto kI = 0;
constexpr auto kD = 0;
}  // namespace PID

namespace FeedForward {
constexpr auto kS = 0;
constexpr auto kV = 0;
constexpr auto kA = 0;  // leave at 0
}  // namespace FeedForward
}  // namespace SelfBalancingConstants

namespace SensorIds {
constexpr int PIDGEON_CAN_ID = 1;
}

namespace RobotValues {
// Speed Scaling
constexpr double TURBO_MODE_SPEED_SCALING = 0.85;
constexpr double NORMAL_MODE_SPEED_SCALING = 0.75;  // 0.75 initially
constexpr double TURTLE_MODE_SPEED_SCALING = 0.35;
}  // namespace RobotValues

namespace MotorIds {
namespace SparkMax {
constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
}  // namespace SparkMax
}  // namespace MotorIds

namespace OperatorInterface {
constexpr int DRIVER_JOYSTICK = 0;

using RateLimit = units::unit_t<
    units::compound_unit<units::scalar, units::inverse<units::seconds>>>;
constexpr RateLimit DRIVER_JOYSTICK_RATE_LIMIT = 2.0 / 1_s;

namespace LogitechGamePad {
// Note: these values were derived from one of the Logitech-branded
// controllers on 22Jan2022. But it looks like there *may* be differences
// between apparently identical devices.... :-(

// Axes - Used with the "getRawAxis()" function to access the data for the
// individual sticks on the controller (e.g., for "tank drive" coding).
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