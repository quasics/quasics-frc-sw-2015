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
namespace CANBusIds {
  namespace SparkMaxIds {
    // The following values reflect the Nike drive base and its idea of
    // "front".
    constexpr int Left_Front_Number = 1;
    constexpr int Left_Rear_Number = 2;
    constexpr int Right_Front_Number = 3;
    constexpr int Right_Rear_Number = 4;
  }  // namespace SparkMaxIds
  namespace VictorSPXIds {
    constexpr int ShootingMotor = 3;
    constexpr int IntakeMotor = 1;
    constexpr int ConveyorMotor = 2;
  }  // namespace VictorSPXIds
  namespace TalonFXIds {
    constexpr int ShootingMotor = 1;
  } // namespace VictorFXIds
  namespace Other {
    // Note: the default CAN bus ID for a PCM is 0, but we've got at least one
    // PCM (currently on the "demo board") that's using an ID of 1.  This has
    // bitten us on the butt a couple of times, since not using the correct ID
    // for the PCM will mean that (a) the compressor won't turn on, and (b) the
    // code will generate run-time errors that it can't "see" the solenoids
    // (when the actual problem is that it can't find the PCM).
    constexpr int PCM = 0;
  }  // namespace Other
}  // namespace CANBusIds

namespace PneumaticsIds {
  constexpr int IntakeSolenoidForward = 0;
  constexpr int IntakeSolenoidBackward = 1;

  constexpr int IntakeSolenoid2Forward = 2;
  constexpr int IntakeSolenoid2Backward = 3;

  // This is passed to the compressor, for use as a default behind the
  // scenes.
  constexpr int DefaultSolenoidId = IntakeSolenoidForward;
}  // namespace PneumaticsIds

namespace PwmIds {
  constexpr int ShooterServo = 1;
}  // namespace PwmIds

namespace DigitalIOMappings {
  constexpr int IntakeLimitSwitch = 0;
  constexpr int ConveyorBeamSensor = 1;
}  // namespace DigitalIOMappings

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
    // The following buttons are below the shoulder, and would be triggers on an
    // Xbox controller.
    constexpr int LeftTriggerButton = 7;
    constexpr int RightTriggerButton = 8;

    // TODO(scott): Check the values for the following.
    constexpr int BackButton = 9;
    constexpr int StartButton = 10;
    constexpr int LeftStickPress = 11;
    constexpr int RightStickPress = 12;
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

namespace DrivebaseConstants {
  constexpr double kNormalSpeedScaling = .6;
  constexpr double kTurtleSpeedScaling = .4;
  constexpr double kTurboSpeedScaling = .8;

  constexpr units::meter_t kTrackWidthMeters{/*1.3965298*/ 1.1972123413926241};
  const frc::DifferentialDriveKinematics kDriveKinematics{kTrackWidthMeters};
  constexpr auto ksVolts = 0.31_V;
  constexpr auto kvVoltSecondsPerMeter = 2.74 * 1_V * 1_s / 1_m;
  constexpr auto kaVoltSecondsSquaredPerMeter = 0.349 * 1_V * 1_s * 1_s / 1_m;
  constexpr double kPDriveVel = 2.28;
  constexpr double kIDriveVel = 0.0;
  constexpr double kDDriveVel = 0.0;
  constexpr auto kMaxSpeed = 0.5_mps;
  constexpr auto kMaxAcceleration = 0.8_mps_sq;
  constexpr double kRamseteB = 2;
  constexpr double kRamseteZeta = 0.7;
}  // namespace DrivebaseConstants

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
  constexpr const char* kPathID("path_id");
}  // namespace NetworkTableNames
