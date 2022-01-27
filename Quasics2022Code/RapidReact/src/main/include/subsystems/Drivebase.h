// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <rev/CANSparkMax.h>
#include "units/length.h"


#include "Constants.h"

class Drivebase : public frc2::SubsystemBase {
 public:

  Drivebase();

  void Periodic() override;

  void Stop(){
    SetMotorPower(0, 0);
  }

  void SetMotorPower(double leftPower, double rightPower);

  units::meter_t  GetLeftEncoders();

  units::meter_t GetRightEncoders();

  void ResetEncoders();

 private:

  rev::CANSparkMax m_leftFront{MotorIds::LEFT_FRONT_DRIVE_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::RIGHT_FRONT_DRIVE_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::LEFT_BACK_DRIVE_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::RIGHT_BACK_DRIVE_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

  //encoders for each of the motors.
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftBackEncoder = m_leftBack.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightBackEncoder = m_rightBack.GetEncoder();

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  std::unique_ptr<frc::DifferentialDrive> m_drive;
};


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

