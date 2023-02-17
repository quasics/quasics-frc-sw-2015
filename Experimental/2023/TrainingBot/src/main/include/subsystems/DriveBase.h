// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/interfaces/Gyro.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>

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
   * Enable/disable "breaking mode" (i.e., lock the motors in place when the power is ~0).
   */
  void EnableBreakingMode(bool tf)
  {
    auto mode = tf ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;
    m_leftFront.SetIdleMode(mode);
    m_rightFront.SetIdleMode(mode);
    m_leftRear.SetIdleMode(mode);
    m_rightRear.SetIdleMode(mode);
  }

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

  units::degree_t GetAngle() { return units::degree_t(m_gyro->GetAngle()); }

  void ResetGyro() { m_gyro->Reset(); }

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

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;
  std::unique_ptr<frc::DifferentialDrive> m_drive;

  // I'm using a pointer to the Gyro base type, rather than creating the actual
  // gyro here, because our different drive bases don't all use the same kind of
  // gyros (which is unfortunate).  Doing it this way means that the code that
  // "knows about" the DriveBase class only sees that we have a (generic) Gyro,
  // and not a specific one.  As a result, if we change the type of Gyro being
  // set up in the .cpp file, nothing *outside* the .cpp file should need to
  // change (or be rebuilt).
  std::unique_ptr<frc::Gyro> m_gyro;
};
