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
            LeftVertical = 1,
            LeftHorizontal = 0,
            LeftTrigger = 2,    // Range is 0-1
            RightVertical = 5,
            RightHorizontal = 4,
            RightTrigger = 3,    // Range is 0-1
        };
        enum Buttons {
            A = 0,
            B = 1,
            X = 2,
            Y = 3,
            G = 6,
            S = 7,
            LeftShoulder = 4,
            RightShoulder = 5,
        };
    }   // GameSirPro
}   // namespace JoystickDefinitions

namespace DriveConstants {
constexpr double kCountsPerRevolution = 1440.0;
constexpr double kWheelDiameterInch = 2.75;
}  // namespace DriveConstants
