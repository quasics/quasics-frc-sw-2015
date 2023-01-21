// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

constexpr int kDriverControllerPort = 0;



}  // namespace OperatorConstants

namespace MotorIds {
  namespace SparkMax {
    constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
    constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
    constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
  }  // namespace SparkMax
}  // namespace MotorIds

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