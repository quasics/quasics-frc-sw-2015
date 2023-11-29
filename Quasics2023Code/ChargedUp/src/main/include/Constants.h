// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include "ConditionalCompileFlags.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// REFERENCE POINTS
// Blue 1: (2.25, 4.75)
// Blue 2: (2.25, 2.75)
// Blue 3: (2.25, 0.75)
// Note: red autonomous commands are mirrored, reference points not needed

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

namespace Intake {
// if intakeDeployment velocity is < STOP_VELOCITY, robot will stop
// intakeDeployment because deployment is completed
constexpr double STOP_VELOCITY = 0;
constexpr int CLOCKS_UNTIL_ABOVE_STOP_VELOCITY = 0;
}  // namespace Intake

namespace Floor {
constexpr double STOP_VELOCITY = 1;
}

namespace AutonomousTeamAndStationPositions {
constexpr auto Blue1 = "Blue 1";
constexpr auto Blue2 = "Blue 2";
constexpr auto Blue3 = "Blue 3";
constexpr auto Red1 = "Red 1";
constexpr auto Red2 = "Red 2";
constexpr auto Red3 = "Red 3";
}  // namespace AutonomousTeamAndStationPositions

namespace AutonomousSpeeds {
constexpr auto DRIVE_SPEED = 0.75;
constexpr auto BRIDGE_DRIVE_SPEED = 0.5;
constexpr auto OVER_CHARGING_STATION_SPEED = 0.5;
constexpr auto SCORE_FLOOR_EJECTION_SPEED = 0.45;
constexpr auto SCORE_FLOOR_EJECTION_TIME = 0.3_s;
constexpr auto DROP_FLOOR_EJECTION_SPEED = 0.2;
constexpr auto DROP_FLOOR_EJECTION_TIME = 0.3_s;
constexpr auto INTAKE_EXTENSION_SPEED = 0.5;
constexpr auto FLOOR_RETRACTION_SPEED = 0.5;
}  // namespace AutonomousSpeeds

namespace DigitalInput {
constexpr int INTAKE_EXTEND_LEFT_LIMIT_SWITCH_ID = 3;
constexpr int FLOOR_RETRACTION_LIMIT_SWITCH_ID = 4;

#ifdef ENABLE_EXPANDED_INTAKE_LIMIT_SWITCHES
constexpr int INTAKE_EXTEND_RIGHT_LIMIT_SWITCH_ID = 5;
constexpr int INTAKE_RETRACT_LEFT_LIMIT_SWITCH_ID = 6;
constexpr int INTAKE_RETRACT_RIGHT_LIMIT_SWITCH_ID = 7;
#endif

constexpr int FLOOR_INTAKE_ENCODER = 2;
}  // namespace DigitalInput

namespace AutonomousSelectedOperation {
const auto DoNothing = "Do nothing";
const auto GTFO = "GTFO";
const auto GTFODock = "GTFO and Dock";
/*
const auto MoveToDefenseAgainstScoringWall =
    "Move to defense against scoring wall";
const auto MoveToDefenseAgainstOuterWall =
    "Move to defense against outer wall";  COMMENT OUT
const auto ScoreAndMoveToDefenseAgainstScoringWall =
    "Score then move to defense against scoring wall";  COMMENT OUT
const auto ScoreAndMoveToDefenseAgainstOuterWall =
    "Score then move to defense against outer wall";  COMMENT OUT
const auto DropAndMoveToDefenseAgainstScoringWall =
    "Drop then move to defense against scoring wall";  COMMENT OUT
const auto DropAndMoveToDefenseAgainstOuterWall =
    "Drop then move to defense against outer wall";  COMMENT OUT
*/
const auto ScoreAndLeave = "Score a Game Piece and GTFO";
const auto ScorePiece = "Score Game Piece";
const auto JustCharge = "Just Charge";
const auto ScoreThenCharge =
    "Score Game Piece then Charge";  // I dont Remmeber if this was tested
const auto ScoreGTFOCharge = "Score Game Piece, GTFO and Charge";
const auto DropGTFOCharge = "Drop Game Piece, GTFO Charge";
const auto ScoreThenEndNearGamePiece = "Score then end near game piece";
const auto DropGamePiece = "Drop Game Piece";
const auto DropThenEndNearGamePiece = "Drop Go out pickup cube";
const auto DropAndGTFO = "Drop Game Piece and Leave";
const auto DropAndCharge = "Drop and Charge";
/*const auto ScoreTwiceThenCharge =
    "Score get another score charge";  // COMMENT OUT
const auto DropTwiceThenCharge =
    "Drop Get Another, Drop, Charge";  // COMMENT OUT*/
const auto DropTwice =
    "Drop Get Another, Drop";  // Good for sides DO NOT USE FOR MIDDLE
// const auto DropThree = "Drop, Pickup, Repeat 3 times";  // COMMENT OUT
}  // namespace AutonomousSelectedOperation

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
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 1.5_mps_sq;

constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

constexpr units::length::meter_t TRACK_WIDTH_METERS_GLADYS = 0.559_m;
const frc::DifferentialDriveKinematics kDriveKinematics(
    TRACK_WIDTH_METERS_GLADYS);
}  // namespace PathWeaverConstants

namespace SelfBalancingConstants {
namespace PID {
  constexpr auto ks = 0;
  constexpr auto kP = 0.025;
  constexpr auto kI = 0.003;
  constexpr auto kD = 0;
}  // namespace PID

namespace FeedForward {
  constexpr auto kS = 0;
  constexpr auto kV = 0;
  constexpr auto kA = 0;  // leave at 0
}  // namespace FeedForward
}  // namespace SelfBalancingConstants

namespace StraightDrivingConstants {
namespace PID {
  constexpr auto kP = 0.02;
  constexpr auto kI = 0.005;
  constexpr auto kD = 0;
}  // namespace PID
}  // namespace StraightDrivingConstants

namespace PIDTurningConstants {
/*
constexpr auto kP = 0.02;
constexpr auto kI = 0.03;
constexpr auto kD = 0;*/

constexpr auto kP = 0.02;
constexpr auto kI = 0.005;
constexpr auto kD = 0;

}  // namespace PIDTurningConstants

namespace SensorIds {
constexpr int PIDGEON_CAN_ID = 1;
}

namespace RobotSpeedScaling {
constexpr double TURBO_MODE_SPEED_SCALING = 1.00;
constexpr double NORMAL_MODE_SPEED_SCALING = 0.80;
constexpr double TURTLE_MODE_SPEED_SCALING = 0.50;
}  // namespace RobotSpeedScaling

namespace IntakeConstants {
namespace RollerSpeeds {
  constexpr double CUBES = 0.60;
  constexpr double CONES = 1.00;
}  // namespace RollerSpeeds
}  // namespace IntakeConstants

namespace PhotonVisionConstants {
namespace CameraAndTargetValues {
  constexpr units::meter_t CAMERA_HEIGHT = 10_in;  // 19.25_in on real bot
  constexpr units::meter_t TARGET_HEIGHT = 17_in;  // 14.25on real field
  constexpr units::radian_t CAMERA_PITCH = 0_rad;
  const units::meter_t GOAL_RANGE_METERS = 2_ft;  // 3ft on real field
}  // namespace CameraAndTargetValues

namespace LinearPID {
  const double kP = 0.07;
  const double kI = 0.0;
  const double kD = 0.0;
}  // namespace LinearPID

namespace AngularPID {
  const double kP = 0.02;
  const double kI = 0.0;
  const double kD = 0.0;
}  // namespace AngularPID

}  // namespace PhotonVisionConstants

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

namespace LightingValues {
constexpr int PORT_NUMBER = 9;
constexpr int TEST_STRIP_PIXEL_COUNT = 14;
constexpr int GLADYS_STRIP_PIXEL_COUNT = 120;  // 2 x 60pixel strips
constexpr int PIXEL_NUMBER = GLADYS_STRIP_PIXEL_COUNT;
}  // namespace LightingValues
