// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.h"

class DriveBase : public frc2::SubsystemBase
{
  // Things that *any* subsystem "knows" how to do
public:
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Stuff specific to our DriveBase subsystem
public:
  /**
   * Set power to left and right sides of the robot.
   *
   * Legal values are -1.0 to +1.0, though a scaling factor will be applied.
   */
  void TankDrive(double leftPercent, double rightPercent);

  /**
   * Stops the robot.
   */
  void Stop() { TankDrive(0, 0); }

  /**
   * Returns the distance traveled by the left wheels since they were last
   * reset.
   */
  units::meter_t GetLeftDistance() { return units::meter_t(m_leftFrontEncoder.GetPosition()); }

  /**
   * Returns the distance traveled by the right wheels since they were last
   * reset.
   */
  units::meter_t GetRightDistance() { return units::meter_t(m_rightFrontEncoder.GetPosition()); }

  /** Returns the current speed of the left-side wheels. */
  units::meters_per_second_t GetLeftVelocity() { return units::meters_per_second_t(m_leftFrontEncoder.GetVelocity()); }

  /** Returns the current speed of the right-side wheels. */
  units::meters_per_second_t GetRightVelocity() { return units::meters_per_second_t(m_rightFrontEncoder.GetVelocity()); }

  void ResetEncoders()
  {
    m_leftFrontEncoder.SetPosition(0);
    m_rightFrontEncoder.SetPosition(0);
  }

  units::degree_t GetAngle();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_leftFront{MotorIds::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftRear{MotorIds::LEFT_REAR_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightRear{MotorIds::RIGHT_REAR_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  // encoders for each of the front motors.
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();

  std::unique_ptr<frc::DifferentialDrive> m_drive;

  ctre::phoenix::sensors::WPI_Pigeon2 m_gyro{SensorIds::PIGEON2_CAN_ID};
};
