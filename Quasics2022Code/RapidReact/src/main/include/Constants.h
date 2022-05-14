// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <string_view>

// Precisely ONE of the following should be enabled (either by saying "#define
// NAME", or uncommenting it), while the others should disbled (by saying
// "#undef NAME" or commenting them out).  This will control which set of values
// are used to control stuff like PID feedback during path following, etc.

#define Target_Sally
#undef Target_Mae
#undef Target_Nike

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
// this is mae's gear ratio of 10.71

constexpr double EXTENSION_SPEED = 1.00;

constexpr double RETRACTION_SPEED = -1.00;

namespace SensorIds {
  constexpr int PIDGEON_CAN_ID = 1;
}

// Constants for scaling driving speeds in teleop mode.
constexpr double TURBO_MODE_SPEED_SCALING = 0.85;
constexpr double NORMAL_MODE_SPEED_SCALING = 0.75;
constexpr double TURTLE_MODE_SPEED_SCALING = 0.35;

constexpr units::length::inch_t TRACK_WIDTH_INCHES_SALLY =
    22.0_in;  // This is track width for Mae. Sally
              // is  22.0_in;

namespace SallyDriverConstants {
  constexpr auto ks = 0.18082_V;
  constexpr auto kv = 1.0444 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.21143 * 1_V * 1_s * 1_s / 1_m;
  constexpr auto kp = 1.4414;
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
    constexpr int REAR_ROLLER_ID = 8;
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
  constexpr int INTAKE_LIMIT_SWITCH_ID = 9;
}  // namespace DigitalInput

namespace LightingValues {
  constexpr int PWM_PORT = 7;
  constexpr int NUM_LIGHTS = 60;
}  // namespace LightingValues

// these are some buttons that seem to have incorrect values below are the
// imported versons of Mr. Healy's button bindings from his 2022MattsToy
/*
namespace OperatorInterface {
  constexpr int DRIVER_JOYSTICK = 0;

  namespace LogitechGamePad {

    constexpr int LEFT_X_AXIS = 0;
    constexpr int LEFT_Y_AXIS = 1;
    constexpr int RIGHT_X_AXIS = 2;
    constexpr int RIGHT_Y_AXIS = 3;

    constexpr int LEFTSHOULDER = 5;   // Turbo Mode
    constexpr int RIGHTSHOULDER = 6;  // Turtle Mode
    constexpr int AButton = 1;
    constexpr int BButton = 2;
    constexpr int XButton = 3;
    constexpr int YButton = 4;  // for switch drive
    constexpr int BACKBUTTON = 7;
    constexpr int STARTBUTTON = 8;
    constexpr int LEFTSTICKPRESS = 9;
    constexpr int RIGHTSTICKPRESS = 10;

  }  // namespace LogitechGamePad
}  // namespace OperatorInterface
*/
namespace OperatorInterface {
  constexpr int DRIVER_JOYSTICK = 0;

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

namespace NetworkTableNames {
  constexpr std::string_view kSettingsTab("Settings");
  constexpr std::string_view kShooterSpeedSliderName("shooterSPeed");
  constexpr std::string_view kRollerSpeedSliderName("rollerSPeed");

  constexpr std::string_view kSensorsTab("Sensors");
  constexpr std::string_view kRobotPose("Pose");
}  // namespace NetworkTableNames