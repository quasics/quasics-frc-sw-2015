// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <rev/CANSparkMax.h>

#include "../../../../Common2021/CommonDriveSubsystem.h"
#include "Constants.h"
#include "sensors/MaxboticsUltrasonicSensor.h"

#define ENABLE_ULTRASONICS

// TODO(mjh): Decide if we want to just use values from 1 encoder per side for
// position/distance, or take an average, or what.
class DriveBase : public CommonDriveSubsystem {
 public:
  DriveBase();

  //
  // Methods defined in frc2::SubsystemBase
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //
  // Methods defined in CommonDriveSubsystem.
 public:
  void Stop() override {
    drive->StopMotor();
  }

  void ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                   bool squareInputs) override;

  void TankDrive(double leftSpeed, double rightSpeed) override;

  void ResetEncoders() override;
  double GetLeftEncoderCount() override;
  double GetRightEncoderCount() override;
  units::meter_t GetRightDistance() override;
  units::meter_t GetLeftDistance() override;

  frc::Gyro& GetZAxisGyro() override {
    return m_adiGyro;
  }

  // Trajectory-following support.
  frc::Pose2d GetPose() override;

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds() override;

  void ResetOdometry(frc::Pose2d pose) override;

  void TankDriveVolts(units::volt_t left, units::volt_t right) override;

  units::meter_t GetTrackWidth() override {
    return kTrackWidthMeters;
  }

  //
  // Methods added in this class.
 public:
  void SetCoastingEnabled(bool enabled);

#ifdef ENABLE_ULTRASONICS
  MaxboticsUltrasonicSensor& GetUltrasonicSensor() {
    return ultrasonic;
  }
#endif  // ENABLE_ULTRASONICS

  //
  // Utility functions used internally by this class.
 private:
  void UpdateOdometry();
  void ConfigureEncoders();

  /** Sets up reporting of drive base data on the Shuffleboard interface. */
  void ConfigureShuffleboard();

  /** Updates reporting of drive base data on the Shuffleboard interface. */
  void UpdateShuffleboard();

  //
  // Data and sub-objects associated with the class.
 private:
  static constexpr units::meter_t kTrackWidthMeters{1.3965298};

  // Our motors
  rev::CANSparkMax m_leftFront;
  rev::CANSparkMax m_leftRear;
  rev::CANSparkMax m_rightFront;
  rev::CANSparkMax m_rightRear;

  // Encoders for each of the motors.
  rev::CANEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::CANEncoder m_leftRearEncoder = m_leftRear.GetEncoder();
  rev::CANEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::CANEncoder m_rightRearEncoder = m_rightRear.GetEncoder();

  // Higher-level interfaces to the motors
  std::unique_ptr<frc::SpeedControllerGroup> leftMotors;
  std::unique_ptr<frc::SpeedControllerGroup> rightMotors;
  std::unique_ptr<frc::DifferentialDrive> drive;

  // Our gyro.
  frc::ADXRS450_Gyro m_adiGyro{
      frc::SPI::Port::kOnboardCS0};  // Chip Select jumper is set to CS0

  // Tracks position/direction for use in trajectory following.
  frc::DifferentialDriveOdometry m_odometry;

  // Used to display information about the drive base on the shuffleboard.
  nt::NetworkTableEntry leftDistance, rightDistance, leftSpeed, rightSpeed,
      rotation;

#ifdef ENABLE_ULTRASONICS
  // Ultrasonic sensor for ranging.
  MaxboticsUltrasonicSensor ultrasonic{AnalogInputs::UltrasonicSensorInput};
#endif  // ENABLE_ULTRASONICS
};
