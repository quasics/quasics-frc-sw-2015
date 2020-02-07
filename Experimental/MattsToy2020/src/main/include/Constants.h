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

namespace DriveBaseConstants {
  constexpr double kStandardPowerScalingFactor = 0.45;
  constexpr double kTurboPowerScalingFactor = 0.45;
}  // namespace DriveBaseConstants

namespace ElevatorConstants {
  constexpr double kRisingPower = 0.40;
  constexpr double kLoweringPower = 0.40;
}  // namespace ElevatorConstants

namespace OIConstants {
  constexpr double kDriveControllerDeadBandSize = 0.015;
  constexpr int kDriverControllerPort = 0;
  constexpr int kOperatorControllerPort = 1;

  // Axes - Used with the "getRawAxis()" function to access the data for the
  // individual sticks on the controller (e.g., for "tank drive" coding).
  constexpr int LogitechGamePad_LeftXAxis = 0;
  constexpr int LogitechGamePad_LeftYAxis = 1;
  constexpr int LogitechGamePad_RightXAxis = 2;
  constexpr int LogitechGamePad_RightYAxis = 5;

  // Buttons
  constexpr int LogitechGamePad_AButton = 2;
  constexpr int LogitechGamePad_BButton = 3;
  constexpr int LogitechGamePad_XButton = 4;
  constexpr int LogitechGamePad_YButton = 1;
  constexpr int LogitechGamePad_LeftShoulder = 5;
  constexpr int LogitechGamePad_RightShoulder = 6;
  constexpr int LogitechGamePad_LeftTrigger = 7;
  constexpr int LogitechGamePad_RightTrigger = 8;
  constexpr int LogitechGamePad_LeftStickPress = 11;
  constexpr int LogitechGamePad_RightStickPress = 12;
  constexpr int LogitechGamePad_StartButton = 9;
  constexpr int LogitechGamePad_SelectButton = 10;

constexpr int XBox_ButtonA = 1; 
constexpr int XBox_ButtonB = 2;  
constexpr int XBox_ButtonX = 3;  
constexpr int XBox_ButtonY = 4;     
constexpr int XBox_LeftButton = 5;   
constexpr int XBox_RightButton = 6;  


constexpr int XBox_LeftXAxis = 0;
constexpr int XBox_LeftYAxis = 1;
constexpr int XBox_RightXAxis = 2;
constexpr int XBox_RightYAxis = 5; 

constexpr int XBox_LeftTrigger = 2;   
constexpr int XBox_RightTrigger =3;  

}  // namespace OIConstants

namespace CANBusConstants {
  namespace SparkMaxIds {
    constexpr int DriveBaseLeftFrontId = 4;
    constexpr int DriveBaseLeftRearId = 3;
    constexpr int DriveBaseRightFrontId = 2;
    constexpr int DriveBaseRightRearId = 1;
  }  // namespace SparkMaxIds

  namespace VictorSpxIds {
    constexpr int ShoulderJointId = 1;

    constexpr int LeftElevatorMotorId = 2;
    constexpr int RightElevatorMotorId = 3;
  }  // namespace VictorSpxIds
}  // namespace CANBusConstants