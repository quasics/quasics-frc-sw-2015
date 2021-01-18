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

namespace JoystickDefinitions {
  namespace GameSirPro {
    enum JoystickAxis {
      LeftHorizontal = 0,
      LeftVertical = 1,
      LeftTrigger = 2,     // Range is [0,1]
      RightTrigger = 3,    // Range is [0,1]
      RightHorizontal = 4,
      RightVertical = 5,
    };
    enum Buttons {
      A = 1,
      B = 2,
      X = 3,
      Y = 4,
      LeftShoulder = 5,
      RightShoulder = 6,
      G = 7,
      S = 8,
    };
  }  // namespace GameSirPro
}  // namespace JoystickDefinitions

namespace DriveConstants {
  constexpr double kCountsPerRevolution = 1440.0;
  constexpr double kWheelDiameterInch = 2.75;
}  // namespace DriveConstants
