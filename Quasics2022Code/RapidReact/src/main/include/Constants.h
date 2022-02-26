// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

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

constexpr double DRIVEBASE_GEAR_RATIO = 8.45;

constexpr double EXTENSION_SPEED = 0.25;

constexpr double RETRACTION_SPEED = -0.25;

constexpr units::length::inch_t TRACK_WIDTH_INCHES_SALLY =
    47.134344149315914763_in;  // This is track width for Mae. Sally
                               // is  22.0_in;

namespace CharacterizationValues {
  constexpr auto ks = 0.31_V;
  constexpr auto kv = 2.74 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.349 * 1_V * 1_s * 1_s / 1_m;
  constexpr auto kp = 2.28;
  constexpr auto ki = 0.0;
  constexpr auto kd = 0.0;
}  // namespace CharacterizationValues

namespace MotorIds {
  namespace SparkMax {
    constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
    constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
    constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
    constexpr int SHOOTER_FLYWHEEL_MOTOR_ID = 5;
    constexpr int LEFT_CLIMBER_MOTOR_ID = 6;
    constexpr int RIGHT_CLIMBER_MOTOR_ID = 7;
  }  // namespace SparkMax
  namespace VictorSPX {
    constexpr int INTAKE_MOTOR_ID = 1;
    constexpr int CONVEYOR_MOTOR_ID = 2;
    constexpr int INTAKE_DEPLOYMENT_MOTOR = 3;
  }  // namespace VictorSPX
}  // namespace MotorIds

namespace DigitalInput {
  constexpr int TOP_LIMIT_SWITCH_ID = 1;
  constexpr int BOTTON_LIMIT_SWITCH_ID = 2;
}  // namespace DigitalInput

namespace LightingValues {
  constexpr int PWM_PORT = 7;
  constexpr int NUM_LIGHTS = 60;
}  // namespace LightingValues
namespace OperatorInterface {
  constexpr int DRIVER_JOYSTICK = 0;

  namespace LogitechGamePad {

    constexpr int LEFT_Y_AXIS = 1;
    constexpr int RIGHT_Y_AXIS = 3;

    constexpr int YButton = 4;        // for switch drive
    constexpr int LEFTSHOULDER = 5;   // Turbo Mode
    constexpr int RIGHTSHOULDER = 6;  // Turtle Mode

  }  // namespace LogitechGamePad
}  // namespace OperatorInterface