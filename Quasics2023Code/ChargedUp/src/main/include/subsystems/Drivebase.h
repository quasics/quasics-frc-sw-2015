// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "units/length.h"
#include "units/velocity.h"

#undef ENABLE_AD_GYRO

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  // values are in percentages from 1 to -1
  void TankDrive(double leftPower, double rightPower);

  void ArcadeDrive(double power, double angle);

  void Stop() { TankDrive(0, 0); }

  void SetBrakingMode(bool enabled);

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

  void ResetEncoders();

  // gets current yaw of robot
  units::degree_t GetAngle();

  /** Returns the robot's current pitch angle (nose pointed up/down). */
  double GetPitch();

  void GyroCalibration();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

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
#ifdef ENABLE_AD_GYRO
  frc::ADXRS450_Gyro m_gyro;
#else
  // ctre::phoenix::sensors::WPI_Pigeon2 m_gyro{SensorIds::PIDGEON_CAN_ID};
#endif
};
