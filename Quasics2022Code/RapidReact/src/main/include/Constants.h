// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

// Precisely ONE of the following should be enabled (either by saying "#define
// NAME", or uncommenting it), while the others should disbled (by saying
// "#undef NAME" or commenting them out).  This will control which set of values
// are used to control stuff like PID feedback during path following, etc.

#undef Target_Sally
#undef Target_Mae
#define Target_Nike

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

constexpr double DRIVEBASE_GEAR_RATIO = 10.71;  // gear ratio for Sally is 8.45
// this is mae's gear ratio of 10.71

constexpr double EXTENSION_SPEED = 0.25;

constexpr double RETRACTION_SPEED = -0.25;

namespace SensorIds {
  constexpr int PIDGEON_CAN_ID = 1;
}

constexpr units::length::inch_t TRACK_WIDTH_INCHES_SALLY =
    47.134344149315914763_in;  // This is track width for Mae. Sally
                               // is  22.0_in;

namespace SallyDriverConstants {
  constexpr auto ks = 0_V;
  constexpr auto kv = 0 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0 * 1_V * 1_s * 1_s / 1_m;
  constexpr auto kp = 0;
  constexpr auto ki = 0;
  constexpr auto kd = 0;
}  // namespace SallyDriverConstants
namespace NikeDriverConstants {
  constexpr auto ks = 0.14961_V;
  constexpr auto kv = 1.3717 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.1627 * 1_V * 1_s * 1_s / 1_m;
  constexpr auto kp = 2.5682;
  constexpr auto ki = 0;
  constexpr auto kd = 0;
}  // namespace NikeDriverConstants
namespace MaeDriverConstants {
  constexpr auto ks = 0.13895_V;
  constexpr auto kv = 1.3143 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.1935 * 1_V * 1_s * 1_s / 1_m;
  constexpr auto kp = 0.0011379;
  constexpr auto ki = 0;
  constexpr auto kd = 0;
}  // namespace MaeDriverConstants

#ifdef Target_Mae
namespace DriverConstants {
  using namespace MaeDriverConstants;
}
#endif

#ifdef Target_Nike
namespace DriverConstants {
  using namespace NikeDriverConstants;
}
#endif

#ifdef Target_Sally
namespace DriverConstants {
  using namespace SallyDriverConstants;
}
#endif

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