/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/util/Color.h>

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
    constexpr int RightXAxis = 4;
    constexpr int RightYAxis = 5;

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

namespace CANBusIds {
  namespace SparkMax {
    // CAN bus IDs for the 4 motors used for the drive base.
    constexpr int Left_Front_No = 4;
    constexpr int Left_Rear_No = 3;
    constexpr int Right_Front_No = 2;
    constexpr int Right_Rear_No = 1;
  }  // namespace SparkMax

  namespace VictorSpx {
    // CAN bus IDs for the motors used in the (floor) intake subsystem
    constexpr int BallIntakeMotor = 1;
    constexpr int ShoulderMotor = 2;

    // CAN bus IDs for the motors used in the "exhaust" subsystem
    constexpr int PushUpMotor = 3;
    constexpr int ShootMotor = 4;

    // CAN bus IDs for the motors used on the climbing subsystem
    constexpr int RightClimberNumber = 5;
    constexpr int LeftClimberNumber = 6;

    // CAN bus IDs for the motors used in command panel manipulator subsystem
    constexpr int SpinMotor = 7;
  }  // namespace VictorSpx
}  // namespace CANBusIds

namespace DriveBaseConstants {
  // Maximum power setting
  constexpr double StandardMaxPower = 0.60;
  constexpr double TurboMaxPower = 0.85;
}  // namespace DriveBaseConstants

namespace CommandPanelConstants {
static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
}  // namespace CommandPanelConstants

namespace ServoConstants {
  constexpr int ServoNumber = 1;
}

namespace PhysicalConstants{
  const int radius = 22;
}