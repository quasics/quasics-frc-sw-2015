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
  constexpr double kTurboPowerScalingFactor = 0.65;
}  // namespace DriveBaseConstants

namespace ElevatorConstants {
  constexpr double kRisingPower = 0.40;
  constexpr double kLoweringPower = 0.40;
}  // namespace ElevatorConstants

namespace OIConstants {
  /// Offset from center on the driver's joysticks that is needed to start drive
  /// wheels moving. (Compensating for calibration issues.)
  constexpr double kDriveControllerDeadBandSize = 0.05;

  /// Joystick port ID for the driver's controller.
  constexpr int kDriverControllerPort = 0;

  /// Joystick port ID for the operator's controller.
  constexpr int kOperatorControllerPort = 1;

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

    constexpr int LeftXAxis = 0;
    constexpr int LeftYAxis = 1;
    constexpr int RightXAxis = 2;
    constexpr int RightYAxis = 5;

    constexpr int LeftTrigger = 2;
    constexpr int RightTrigger = 3;
  }  // namespace XBox
}  // namespace OIConstants

// IDs for devices connected via PWM.
//
// Note: these IDs must be unique.
namespace PwmAssignments {
  constexpr int CameraSpinnerId = 0;
  constexpr int LedStripId = 1;
}  // namespace PwmAssignments

// IDs for devices on the CAN bus.
//
// Note: CAN allows multiple devices to have the same IDs, as long as they are
// different types of hardware.
namespace CANBusConstants {

  namespace SparkMaxIds {
    constexpr int DriveBaseLeftFrontId = 3;
    constexpr int DriveBaseLeftRearId = 4;
    constexpr int DriveBaseRightFrontId = 1;
    constexpr int DriveBaseRightRearId = 2;
  }  // namespace SparkMaxIds

  namespace VictorSpxIds {
    constexpr int ShoulderJointId = 1;

    constexpr int LeftElevatorMotorId = 2;
    constexpr int RightElevatorMotorId = 3;

  }  // namespace VictorSpxIds
}  // namespace CANBusConstants