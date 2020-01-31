/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
namespace OIConstants {
    // Axes - Used with the "getRawAxis()" function to access the data for the

// individual sticks on the controller (e.g., for "tank drive" coding).

constexpr int LogitechGamePad_LeftXAxis = 0;

constexpr int LogitechGamePad_LeftYAxis = 1;

constexpr int LogitechGamePad_RightXAxis = 2;

constexpr int LogitechGamePad_RightYAxis = 5;
}