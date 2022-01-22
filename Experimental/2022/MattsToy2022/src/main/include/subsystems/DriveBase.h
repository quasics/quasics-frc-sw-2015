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

  void ResetEncoders();
  units::meter_t GetRightDistance();
  units::meter_t GetLeftDistance();
  units::meters_per_second_t GetLeftSpeed();
  units::meters_per_second_t GetRightSpeed();

 private:
  void ConfigureEncoders();

 private:
  rev::CANSparkMax leftFront{MotorIds::LEFT_FRONT_DRIVE_MOTOR_ID,
                             rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightFront{MotorIds::RIGHT_FRONT_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftRear{MotorIds::LEFT_REAR_DRIVE_MOTOR_ID,
                            rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightRear{MotorIds::RIGHT_REAR_DRIVE_MOTOR_ID,
                             rev::CANSparkMax::MotorType::kBrushless};

  // Encoders for each of the motors.
  rev::SparkMaxRelativeEncoder leftFrontEncoder = leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::SparkMaxRelativeEncoder rightRearEncoder = rightRear.GetEncoder();

  std::unique_ptr<frc::MotorControllerGroup> leftSide;
  std::unique_ptr<frc::MotorControllerGroup> rightSide;

  std::unique_ptr<frc::DifferentialDrive> drive;
};
