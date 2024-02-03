// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <numbers>

#include "ConditionalCompileFlags.h"
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

namespace RobotPhysics {
  constexpr double DRIVEBASE_GEAR_RATIO = 8.45;  // TODO: add mae gear ratio
  constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;
}  // namespace RobotPhysics

namespace OperatorConstants {

  namespace LogitechGamePad {
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    // Note that the left and right triggers aren't treated as buttons: they
    // report to the driver's station software as if they're joysticks (with a
    // range of [0.0, 1.0], unlike regular joysticks).
    constexpr int LeftXAxis = 0;
    constexpr int LeftYAxis = 1;
    constexpr int LeftTriggerAxis = 2;
    constexpr int RightTriggerAxis = 3;
    constexpr int RightXAxis = 2;
    constexpr int RightYAxis = 3;

    // Buttons
    constexpr int AButton = 1;
    constexpr int BButton = 2;
    constexpr int XButton = 3;
    constexpr int YButton = 4;
    constexpr int LeftShoulder = 5;
    constexpr int RightShoulder = 6;
    constexpr int BackButton = 7;
    constexpr int StartButton = 8;
    constexpr int LeftStickPress = 9;
    constexpr int RightStickPress = 10;
  }  // namespace LogitechGamePad

}  // namespace OperatorConstants

namespace PathWeaverConstants {
#ifdef USING_MAE
  constexpr auto kS = 0.13895_V;
  constexpr auto kV = 1.3143 * (1_V * 1_s / 1_m);
  constexpr auto kA = 0.1935 * (1_V * 1_s * 1_s / 1_m);
  constexpr double kP = 0.001379;
  constexpr double kI = 0;
  constexpr double kD = 0;
#endif
#ifdef USING_SALLY
  constexpr auto kS = 0.19529_V;
  constexpr auto kV = 2.2329 * (1_V * 1_s / 1_m);
  constexpr auto kA = 0.36638 * (1_V * 1_s * 1_s / 1_m);
  constexpr double kP = 0.29613;
  constexpr double kI = 0;
  constexpr double kD = 0;
#endif
}  // namespace PathWeaverConstants

namespace MotorIds {
  namespace SparkMax {
    constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
    constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
    constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
    constexpr int LEFT_CLIMBER_MOTOR_ID = 5;
    constexpr int RIGHT_CLIMBER_MOTOR_ID = 6;  // is inverted
    constexpr int INTAKE_DEPLOYMENT_MOTOR = 7;
    constexpr int INTAKE_MOTOR = 8;
    constexpr int SHOOTER_FLYWHEEL_MOTOR_LEADER_ID = 9;
  }  // namespace SparkMax

}  // namespace MotorIds

namespace MotorSpeeds {
  constexpr double EXTENSION_SPEED = 1.00;

  constexpr double RETRACTION_SPEED = -1.00;
}  // namespace MotorSpeeds

namespace RobotSpeedScaling {
  constexpr double TURBO_MODE_SPEED_SCALING = 0.90;
  constexpr double NORMAL_MODE_SPEED_SCALING = 0.60;
  constexpr double TURTLE_MODE_SPEED_SCALING = 0.30;
}  // namespace RobotSpeedScaling

// DIO ports 0-9 are on-board, 10-25 are on the MXP port.
namespace DigitalInput {
  constexpr int INTAKE_EXTEND_LIMIT_SWITCH_ID = 8;
  constexpr int INTAKE_RETRACT_LIMIT_SWITCH_ID = 9;

  constexpr int SIMULATED_LEFT_ENCODER_A_PORT = 10;
  constexpr int SIMULATED_LEFT_ENCODER_B_PORT = 11;
  constexpr int SIMULATED_RIGHT_ENCODER_A_PORT = 12;
  constexpr int SIMULATED_RIGHT_ENCODER_B_PORT = 13;
}  // namespace DigitalInput

// PWM ports 0-9 are on-board, 10-19 are on the MXP port.
namespace PWMPorts {
  constexpr int LIGHTING_PORT = 1;
  constexpr int SIMULATED_LEFT_MOTOR_PORT = 18;
  constexpr int SIMULATED_RIGHT_MOTOR_PORT = 19;
}  // namespace PWMPorts

namespace SensorIds {
  constexpr int PIGEON_CAN_ID = 1;
}

/* namespace LightingValues {
  constexpr int PORT_NUMBER = fill in;
  constexpr int STRIP_PIXEL_COUNT = fill in;
  constexpr int PIXEL_NUMBER = STRIP_PIXEL_COUNT;
} */
// namespace LightingValues

constexpr units::length::meter_t TRACK_WIDTH_METERS_SALLY = 0.5588_m;
const frc::DifferentialDriveKinematics kDriveKinematics(
    TRACK_WIDTH_METERS_SALLY);

constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 1_mps_sq;

constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

namespace RobotConstants {
  static constexpr units::meters_per_second_t MAX_SPEED{3.0};
  static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{
      std::numbers::pi};
}  // namespace RobotConstants

using RateLimit = units::unit_t<
    units::compound_unit<units::scalar, units::inverse<units::seconds>>>;

/** Limit on robot acceleration. */
constexpr RateLimit DRIVER_JOYSTICK_RATE_LIMIT = 1 / 1_s;

namespace AutonomousTeamAndStationPositions {
  constexpr auto inFrontOfAmp = "In front of amp";          // 1A
  constexpr auto leftOfSpeaker = "Left of speaker";         // 1B
  constexpr auto inFrontOfSpeaker = "In front of speaker";  // 2
  constexpr auto rightOfSpeaker = "Right of speaker";       // 3A
  constexpr auto farField = "Far field";                    // 3B
}  // namespace AutonomousTeamAndStationPositions

namespace AutonomousSelectedOperation {
  const auto doNothing = "Do nothing";
  const auto GTFO = "GTFO";
  const auto score1 = "Score 1 piece";
  const auto score1GTFO = "Score 1 piece, GTFO";
  const auto score2 = "Score 2 piece";
  const auto score2GTFO = "Score 2 piece, GTFO";
  const auto score3 = "Score 3 piece";
  const auto score3GTFO = "Score 3 piece, GTFO";
}  // namespace AutonomousSelectedOperation

namespace AutonomousScoreDestinations {
  const auto none = "None";
  const auto amp = "Amp";
  const auto leftOfSpeaker = "Left of speaker";
  const auto inFrontOfSpeaker = "In front of speaker";
  const auto rightOfSpeaker = "Right of speaker";
}  // namespace AutonomousScoreDestinations

namespace ShootingSpeeds {
  const double amp = 0.5;
  const double speaker = 1.0;
}