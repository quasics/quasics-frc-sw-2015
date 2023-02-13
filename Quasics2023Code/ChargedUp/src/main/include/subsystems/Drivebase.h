// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <frc/drive/DifferentialDrive.h>
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

  // TODO(ethan/josh): Add docs to clarify what angle this is.  (I'm assuming
  // it's "yaw", but it would be good to make this explicit.)
  units::degree_t GetAngle();

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

  // TODO(matthew): Add documentation for what this represents/does. DONE

  // Because the Pigeon(gyro) on calibration does not fully reset itself. Its
  // output is shifted so that it accurately represents the situation For
  // Example: the reading of pitch was -4.something even though the robot was
  // flat. Thus the output was shifted to 0
  double m_pitchShift = 0;

  ctre::phoenix::sensors::WPI_Pigeon2 m_gyro{SensorIds::PIDGEON_CAN_ID};
};
