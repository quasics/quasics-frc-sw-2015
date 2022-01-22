// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class DriveBase : public frc2::SubsystemBase {
 public:
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetCoastingEnabled(bool tf);
  void TankDrive(double leftSpeed, double rightSpeed);
  void Stop() {
    TankDrive(0, 0);
  }

  void ResetEncoders();
  units::meter_t GetRightDistance();
  units::meter_t GetLeftDistance();
  units::meters_per_second_t GetLeftSpeed();
  units::meters_per_second_t GetRightSpeed();

 private:
  void ConfigureEncoders();

 private:
  rev::CANSparkMax m_leftFront{MotorIds::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftRear{MotorIds::LEFT_REAR_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightRear{MotorIds::RIGHT_REAR_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  // Encoders for each of the motors.
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftRearEncoder = m_leftRear.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightRearEncoder = m_rightRear.GetEncoder();

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  std::unique_ptr<frc::DifferentialDrive> m_drive;
};
