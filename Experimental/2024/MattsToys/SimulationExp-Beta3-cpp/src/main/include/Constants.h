#pragma once

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
