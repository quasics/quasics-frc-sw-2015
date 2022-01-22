// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * @file
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

constexpr units::length::inch_t WHEEL_DIAMETER_INCHES = 6.0_in;

/// Gear ratio used for the 2020/2021 robots.
constexpr double DRIVE_BASE_GEAR_RATIO_2021 = 10.71;
constexpr double DRIVE_BASE_GEAR_RATIO_2022 = 8.45;
constexpr double DRIVE_BASE_GEAR_RATIO = DRIVE_BASE_GEAR_RATIO_2022;

namespace MotorIds {
  constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
  constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 2;
  constexpr int LEFT_REAR_DRIVE_MOTOR_ID = 3;
  constexpr int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
}  // namespace MotorIds

namespace OperatorInterface {
  constexpr int DRIVER_JOYSTICK = 0;

  namespace LogitechGamePad {
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    // Note that the left and right triggers aren't treated as buttons: they
    // report to the driver's station software as if they're joysticks (with a
    // range of [0.0, 1.0], unlike regular joysticks).
    constexpr int LEFT_X_AXIS = 0;
    constexpr int LEFT_Y_AXIS = 1;
    constexpr int RIGHT_X_AXIS = 2;
    constexpr int RIGHT_Y_AXIS = 3;

    // TODO: These are *clearly* wrong, and need to be fixed.
    constexpr int LEFT_TRIGGER_AXIS = 2;
    constexpr int RIGHT_TRIGGER_AXIS = 3;

    // Buttons
    constexpr int A_BUTTON = 1;
    constexpr int B_BUTTON = 2;
    constexpr int X_BUTTON = 3;
    constexpr int Y_BUTTON = 4;
    constexpr int LEFT_SHOULDER = 5;
    constexpr int RIGHT_SHOULDER = 6;
    constexpr int BACK_BUTTON = 7;
    constexpr int START_BUTTON = 8;
    constexpr int LEFT_STICK_PRESS = 9;
    constexpr int RIGHT_STICK_PRESS = 10;
  }  // namespace LogitechGamePad
}  // namespace OperatorInterface

namespace LightingValues {
  constexpr int PWM_PORT = 7;
  constexpr int NUM_LIGHTS = 60;
}  // namespace LightingValues

namespace Deadbands {
  constexpr double DRIVING = 0.055;
}
