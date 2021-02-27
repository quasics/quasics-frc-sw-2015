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
namespace CANBusIds{
    namespace SparkMaxIds{
        constexpr int Left_Front_Number = 4;
        constexpr int Left_Rear_Number = 3;
        constexpr int Right_Front_Number = 2;
        constexpr int Right_Rear_Number = 1;
    }
    namespace VictorSPXIds{
      constexpr int ShootingMotor = 5;
      constexpr int IntakeMotor = 6;
    }
}

namespace OIConstants {
  namespace LogitechGamePad {
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    // Note that the left and right triggers aren't treated as buttons: they
    // report to the driver's station software as if they're joysticks (with a
    // range of [0.0, 1.0], unlike regular joysticks).
    constexpr int LeftXAxis = 0;
    constexpr int LeftYAxis = 1;
    constexpr int LeftTriggerAxis = 2;
    constexpr int RightTriggerAxis = 3;
    constexpr int RightXAxis = 2;
    constexpr int RightYAxis = 3;

    // Buttons
    constexpr int AButton = 1;
    constexpr int BButton = 2;
    constexpr int XButton = 3;
    constexpr int YButton = 4;
    constexpr int LeftShoulder = 5;
    constexpr int RightShoulder = 6;
    constexpr int BackButton = 7;
    constexpr int StartButton = 8;
    constexpr int LeftStickPress = 9;
    constexpr int RightStickPress = 10;
  }  // namespace LogitechGamePad

  namespace XBox {
    constexpr int ButtonA = 1;
    constexpr int ButtonB = 2;
    constexpr int ButtonX = 3;
    constexpr int ButtonY = 4;
    constexpr int LeftButton = 5;
    constexpr int RightButton = 6;
    constexpr int BackButton = 7;
    constexpr int StartButton = 8;

    constexpr int LeftXAxis = 0;
    constexpr int LeftYAxis = 1;
    constexpr int RightXAxis = 2;
    constexpr int RightYAxis = 5;

    constexpr int LeftTrigger = 2;
    constexpr int RightTrigger = 3;
  }  // namespace XBox

  // "Dead band" values for the drive joysticks
  constexpr double DeadBand_LowValue = -0.055;
  constexpr double DeadBand_HighValue = +0.055;
}  // namespace OIConstants

namespace DrivebaseConstants{
  constexpr double powerScaling = .6;
}