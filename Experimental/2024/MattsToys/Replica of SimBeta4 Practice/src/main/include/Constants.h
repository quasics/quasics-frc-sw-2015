// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace OperatorConstants {
  inline constexpr int kDriverControllerPort = 0;
}  // namespace OperatorConstants

namespace LedConstants {
  constexpr int LED_PWM_PORT = 7;
  constexpr int LED_STRIP_SIZE = 40;
}  // namespace LedConstants

namespace MotorIds {
  namespace SparkMax {
    constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
    constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
    constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
  }  // namespace SparkMax
}  // namespace MotorIds

