// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>

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

namespace RobotPhysics {
constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;
constexpr double DRIVEBASE_GEAR_RATIO_MAE = 10.71;
constexpr double DRIVEBASE_GEAR_RATIO_SALLY = 8.45;
constexpr double DRIVEBASE_GEAR_RATIO_GLADYS = 8.45;

// Used to isolate the gear ratio for the robot we're actually building for,
// from the various "known values" for each of the possible robots.
constexpr double DRIVEBASE_GEAR_RATIO = DRIVEBASE_GEAR_RATIO_GLADYS;
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

namespace DigitalInput {
constexpr int INTAKE_EXTEND_LEFT_LIMIT_SWITCH_ID = 1;
constexpr int INTAKE_EXTEND_RIGHT_LIMIT_SWITCH_ID = 2;
constexpr int INTAKE_RETRACT_LEFT_LIMIT_SWITCH_ID = 3;
constexpr int INTAKE_RETRACT_RIGHT_LIMIT_SWITCH_ID = 4;
}  // namespace DigitalInput

namespace AutonomousSelectedOperation {
const auto DoNothing = "Do nothing";
const auto GTFO = "GTFO";
const auto GTFODock = "GTFO and Dock";
const auto MoveToDefenseAgainstScoringWall =
    "Move to defense against scoring wall";
const auto MoveToDefenseAgainstOuterWall = "Move to defense against outer wall";
const auto ScoreAndMoveToDefenseAgainstScoringWall =
    "Score then move to defense against scoring wall";
const auto ScoreAndMoveToDefenseAgainstOuterWall =
    "Score then move to defense against outer wall";
const auto DropAndMoveToDefenseAgainstScoringWall =
    "Drop then move to defense against scoring wall";
const auto DropAndMoveToDefenseAgainstOuterWall =
    "Drop then move to defense against outer wall";
const auto ScoreAndLeave = "Score a Game Piece and GTFO";
const auto ScorePiece = "Score Game Piece";
const auto JustCharge = "Just Charge";
const auto ScoreThenCharge = "Score Game Piece then Charge";
const auto ScoreGTFOCharge = "Score Game Piece, GTFO and Charge";
const auto ScoreThenEndNearGamePiece = "Score then end near game piece";
const auto DropGamePiece = "Drop Game Piece";
const auto DropAndGTFO = "Drop Game Piece and Leave";
const auto DropAndCharge = "Drop and Charge";
}  // namespace AutonomousSelectedOperation

namespace SelfBalancingConstants {
namespace PID {
  constexpr auto ks = 0;
  constexpr auto kP = 0.025;
  constexpr auto kI = 0.005;
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
  constexpr auto kP = 0.025;
  constexpr auto kI = 0.005;
  constexpr auto kD = 0;
}  // namespace PID
}  // namespace StraightDrivingConstants

namespace SensorIds {
constexpr int PIDGEON_CAN_ID = 1;
}

namespace RobotSpeedScaling {
constexpr double TURBO_MODE_SPEED_SCALING = 0.55;
constexpr double NORMAL_MODE_SPEED_SCALING = 0.70;
constexpr double TURTLE_MODE_SPEED_SCALING = 0.35;
}  // namespace RobotSpeedScaling

namespace IntakeConstants {
namespace RollerSpeeds {
  constexpr double FORWARD = 0.85;
  constexpr double BACKWARD = -0.85;

}  // namespace RollerSpeeds
}  // namespace IntakeConstants

namespace PhotonVisionConstants {
namespace CameraAndTargetValues {
  constexpr units::meter_t CAMERA_HEIGHT = 8_in;   // 19.25_in on real bot
  constexpr units::meter_t TARGET_HEIGHT = 12_in;  // 14.25on real field
  constexpr units::radian_t CAMERA_PITCH = 0_rad;
  const units::meter_t GOAL_RANGE_METERS = 2_ft;  // 3ft on real field
}  // namespace CameraAndTargetValues

namespace LinearPID {
  const double kP = 0.01;
  const double kI = 0.0;
  const double kD = 0.0;
}  // namespace LinearPID

namespace AngularPID {
  const double kP = 0.01;
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

namespace OperatorConstants {
/** Joystick port for the driver's controller. */
// CODE_REVIEW: This is a duplicate constant, and should be cleaned up. (mjh)
// See DRIVER_JOYSTICK, below.
constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants

namespace OperatorInterface {
// CODE_REVIEW: This is a duplicate constant, and should be cleaned up. (mjh)
// See kDriverControllerPort, above.
constexpr int DRIVER_JOYSTICK = 0;

using RateLimit = units::unit_t<
    units::compound_unit<units::scalar, units::inverse<units::seconds>>>;

/** Limit on robot accessleration. */
constexpr RateLimit DRIVER_JOYSTICK_RATE_LIMIT = 1 / 1_s;

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

namespace LightingValues {
constexpr int PORT_NUMBER = 9;
constexpr int PIXEL_NUMBER = 14;
}  // namespace LightingValues