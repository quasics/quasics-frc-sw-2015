// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

namespace CANBusIds {
  /**
   * CAN ID for the PCM.
   *
   * Note that the PCM <em>usually</em> has the default ID of 0, and it is
   * generally left that way.  (And some of the WPILib classes are written
   * with that assumption in place.)  However, the PCM on our "pneumatics
   * demo board" was changed at some point to use CAN ID 1, and so we're
   * rolling with that for now.
   *
   * Besides: using a non-standard ID arguably makes this sample code a
   * little more robust, so....  Why not?
   */
  constexpr int PCM = 1;
}  // namespace CANBusIds

namespace PneumaticIds {
  constexpr int Compressor = 0;
  constexpr int IntakeSolenoidForward = 1;
  constexpr int IntakeSolenoidBackward = 2;
}  // namespace PneumaticIds
