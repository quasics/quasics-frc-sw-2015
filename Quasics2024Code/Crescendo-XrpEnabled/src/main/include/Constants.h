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

  inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace MotorIds {
  namespace SparkMax {
    constexpr int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    constexpr int LEFT_BACK_DRIVE_MOTOR_ID = 2;
    constexpr int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
    constexpr int RIGHT_BACK_DRIVE_MOTOR_ID = 4;
    constexpr int LEFT_CLIMBER_MOTOR_ID = 5;
    constexpr int RIGHT_CLIMBER_MOTOR_ID = 6;
  }  // namespace SparkMax

  namespace VictorSPX {
    constexpr int INTAKE_MOTOR_ROLLER_ID = 1;
  }
}  // namespace MotorIds

namespace MotorSpeeds {
  constexpr double EXTENSION_SPEED = 1.00;

  constexpr double RETRACTION_SPEED = -1.00;
}  // namespace MotorSpeeds