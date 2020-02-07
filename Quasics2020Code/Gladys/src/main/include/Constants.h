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
  // Axes - Used with the "getRawAxis()" function to access the data for the
  // individual sticks on the controller (e.g., for "tank drive" coding).
  constexpr int LogitechGamePad_LeftXAxis = 0;
  constexpr int LogitechGamePad_LeftYAxis = 1;
  constexpr int LogitechGamePad_RightXAxis = 2;
  constexpr int LogitechGamePad_RightYAxis = 5;

  // "Dead band" values for the drive joysticks
  constexpr double DeadBand_LowValue = -0.01;
  constexpr double DeadBand_HighValue = +0.01;
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
    // CAN bus IDs for the motors used in the intake subsystem
    constexpr int BallIntakeMotor = 1;
    constexpr int ShoulderMotor = 2;

    // CAN bus IDs for the motors used in command panel manipulator subsystem
    constexpr int SpinMotor = 7;
  }  // namespace VictorSpx
}  // namespace CANBusIds

namespace DriveBaseConstants {
    // Maximum power setting
    constexpr double StandardMaxPower = 0.45;
    constexpr double TurboMaxPower = 0.65;
}  // namespace DriveBaseConstants

namespace CommandPanelConstants {
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
}  // namespace CommandPanelConstants>>>>>>> .r1415
