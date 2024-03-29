// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ConditionalCompileFlags.h"

#ifdef ENABLE_PIGEON
#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#endif  // ENABLE_PIGEON

#ifdef ENABLE_AD_GYRO
#include <frc/ADXRS450_Gyro.h>
#endif  // ENABLE_AD_GYRO

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "units/length.h"
#include "units/velocity.h"

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  // values are in percentages from 1 to -1
  void TankDrive(double leftPower, double rightPower);

  void ArcadeDrive(double power, double angle);

  void TankDriveVolts(units::volt_t left, units::volt_t right);

  void Stop() { TankDrive(0, 0); }

  void SetBrakingMode(bool enabled);
  bool IsInBrakingMode() { return m_isBraking; }

  /**
   * Returns the distance traveled by the left wheels since they were last
   * reset.
   */

  void ConfigureEncoders();

  units::meter_t GetLeftDistance();

  /**
   * Returns the distance traveled by the right wheels since they were last
   * reset.
   */
  units::meter_t GetRightDistance();

  /** Returns the current speed of the left-side wheels. */
  units::meters_per_second_t GetLeftVelocity();

  /** Returns the current speed of the right-side wheels. */
  units::meters_per_second_t GetRightVelocity();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void ResetEncoders();

  // gets current yaw of robot
  units::degree_t GetYaw();

  frc::Pose2d GetPose();

  void ResetOdometry(frc::Pose2d pose);

  /** Returns the robot's current pitch angle (nose pointed up/down). */
  double GetPitch() {
    return GetPitchImpl();
    // return GetRollImpl();
  }

  /** Returns the robot's current roll angle. */
  double GetRoll() {
    // return GetPitchImpl();
    return GetRollImpl();
  }

  void GyroCalibration();

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  /** Returns the robot's current pitch angle (nose pointed up/down). */
  double GetPitchImpl();

  /** Returns the robot's current roll angle. */
  double GetRollImpl();

 private:
  // Individual motors.
  rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  // encoders for each of the motors.
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftBackEncoder = m_leftBack.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightBackEncoder = m_rightBack.GetEncoder();

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  std::unique_ptr<frc::DifferentialDrive> m_drive;

  bool m_isBraking = false;  // To be adjusted in the constructor

  /**
   * Used to adjust the values reported for "pitch", in order to account for the
   * fact that the Pigeon may not be mounted perfectly level on the robot.  When
   * calls are made to GetPitch(), this value will be subtracted from the value
   * that is reported by the Pigeon.
   *
   * Note: this *does* assume that the robot code is restarted when the bot is
   * on a level surface.  If this isn't the case, then the value would need to
   * be reset (again) at some future point.
   */
  double m_pitchShift = 0;
  double m_rollShift = 0;
#ifdef ENABLE_AD_GYRO
  frc::ADXRS450_Gyro m_adGyro;
#elif defined(ENABLE_PIGEON)
  ctre::phoenix::sensors::WPI_Pigeon2 m_pigeon{SensorIds::PIDGEON_CAN_ID};
#endif

  frc::DifferentialDriveOdometry m_odometry{
      0_rad, 0_m, 0_m};  // m_odometry delcared somehwere ELSE?!?!?!
};
