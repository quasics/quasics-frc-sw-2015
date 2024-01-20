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

namespace OperatorConstants {

  inline constexpr int kDriverControllerPort = 0;

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
    constexpr int RIGHT_CLIMBER_MOTOR_ID = 6;
    constexpr int INTAKE_DEPLOYMENT_MOTOR = 1;
    constexpr int SHOOTER_FLYWHEEL_MOTOR_ID = 1;
    constexpr int REAR_ROLLER_ID = 1;
  }  // namespace SparkMax

  namespace VictorSPX {
    constexpr int INTAKE_MOTOR_ROLLER_ID = 1;
  }
}  // namespace MotorIds

namespace MotorSpeeds {
  constexpr double EXTENSION_SPEED = 1.00;

  constexpr double RETRACTION_SPEED = -1.00;
}  // namespace MotorSpeeds

namespace DigitalInput {
  constexpr int INTAKE_EXTEND_LIMIT_SWITCH_ID = 1;
  constexpr int INTAKE_RETRACT_LIMIT_SWITCH_ID = 2;
}  // namespace DigitalInput

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