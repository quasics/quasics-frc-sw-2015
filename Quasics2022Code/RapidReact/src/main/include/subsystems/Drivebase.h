// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "units/length.h"
#include "units/velocity.h"

/**
 * Drive base subsystem, used for movement/navigation.
 */
class Drivebase : public frc2::SubsystemBase {
 public:
  // Constructor.
  Drivebase();

  /**
   * Sets the speeds for the left and right motors to the specified percentages
   * (-1.0 == full reverse, +1.0 == full forward, 0 = stopped).
   *
   * @param leftPower  desired % power for the left motors
   * @param rightPower desired % power for the right motors
   */
  void SetMotorPower(double leftPower, double rightPower);

  /** Convenience method to stop the drive base. */
  void Stop() {
    SetMotorPower(0, 0);
  }

  /**
   * Returns the distance traveled by the left wheels since they were last
   * reset.
   */
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

  /** Resets the encoders used to report distances for left/right wheels. */
  void ResetEncoders();

  // Standard functions for subsystems.
 public:
  void Periodic() override;

  // Internal functions.
 private:
  /** Sets up the encoders to use meaningful units for distance/speed. */
  void ConfigureEncoders();

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

  // Motor controller groups, used to "gang together" motors for a given side.
  //
  // Note that we need to use pointers (rather than direct creation here), since
  // any motor inversions must be configured prior to adding them to a motor
  // controller group.
  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  // Standard WPILib differential drive, used to control left/right motors as a
  // set (e.g., under tank or arcade drive).
  //
  // Since we're creating the controller groups "late" in the construction
  // (using "new"), we need to do the same thing for this (which uses them).
  std::unique_ptr<frc::DifferentialDrive> m_drive;

  // Defines the Gyro in CS0
  frc::ADXRS450_Gyro m_gyro;
  frc::DifferentialDriveOdometry m_odometry{0_rad};
};
