// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

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
      LeftTrigger = 2,   // Range is [0,1]
      RightTrigger = 3,  // Range is [0,1]
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
}  // namespace JoystickDefinitions

namespace DriveConstants {
  constexpr double kCountsPerRevolution = 1440.0;
  constexpr double kWheelDiameterInch = 2.75;
}  // namespace DriveConstants

namespace NetworkTableNames {
  constexpr const char* kVisionTable("Vision");
  constexpr const char* kTargetXEntry("target_x");

  constexpr const char* kVisionSettingsTable("VisionSettings");
  constexpr const char* kLowHSetting("Low_H");
  constexpr const char* kLowSSetting("Low_S");
  constexpr const char* kLowVSetting("Low_V");
  constexpr const char* kHighHSetting("High_H");
  constexpr const char* kHighSSetting("High_S");
  constexpr const char* kHighVSetting("High_V");
}  // namespace NetworkTableNames

namespace RobotData {
  constexpr units::meter_t kTrackwidthMeters{0.142072613};

  namespace DriveConstants {
    const frc::DifferentialDriveKinematics kDriveKinematics{
        RobotData::kTrackwidthMeters};

    // The following values are drawn from drive characterization performed
    // on the Romi "Matt's Toy", and captured in the team wiki here:
    // https://github.com/quasics/quasics-frc-sw-2015/wiki/Scott's-Toy-Constants
    constexpr auto ksVolts = 1.16_V;                                // "kS"
    constexpr auto kvVoltSecondsPerMeter = 1.79 * 1_V * 1_s / 1_m;  // "kV"
    constexpr auto kaVoltSecondsSquaredPerMeter =                   // "kA"
        0.0312 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kPDriveVel =
        0.636;  // "kP" - proportional coefficient for PID
    constexpr double kIDriveVel = 0.0;  // "kI" - integral coefficient for PID
    constexpr double kDDriveVel = 0.0;  // "kD" - derivative coefficient for PID

  }  // namespace DriveConstants

  namespace PathFollowingLimits {
    constexpr auto kMaxSpeed = 0.8_mps;
    constexpr auto kMaxAcceleration = 0.8_mps_sq;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds.
    constexpr double kRamseteB =
        2;  // Tuning parameter (b > 0) for which larger values make
            // convergence more aggressive like a proportional term.
    constexpr double kRamseteZeta =
        0.7;  // Tuning parameter (0 < zeta < 1) for which larger values provide
              // more damping in response.
  }  // namespace PathFollowingLimits
}  // namespace RobotData
