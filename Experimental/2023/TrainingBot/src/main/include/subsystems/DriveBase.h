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
 // Things that any subsystem "knows" how to do
 public:
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 // Stuff specific to our DriveBase subsystem
 public:
  void TankDrive(double leftPercent, double rightPercent);

  void Stop() { TankDrive(0, 0); }

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

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  std::unique_ptr<frc::DifferentialDrive> m_drive;
};
