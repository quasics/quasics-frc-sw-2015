// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h> 
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <units/length.h>



class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Stop();
  void SetMotorSpeed(double leftSpeed, double rightSpeed);
  frc::Gyro& GetZAxisGyro() {
    return adiGyro;
  }
  void ResetEncoders();
  double GetLeftEncoderCount();
  double GetRightEncoderCount();
  units::meter_t GetRightEncoderDistance();
  units::meter_t GetLeftEncoderDistance();


 public:
  frc::Pose2d GetPose();
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  void ResetOdometry(frc::Pose2d pose);
  void TankDriveVolts(units::volt_t left, units::volt_t right);


   
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

   frc::ADXRS450_Gyro adiGyro{
      frc::SPI::Port::kOnboardCS0};

  rev::CANEncoder leftFrontEncoder = leftFront.GetEncoder();
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();

  std::unique_ptr<frc::SpeedControllerGroup> LeftMotors;
  std::unique_ptr<frc::SpeedControllerGroup> RightMotors;
  // frc::SpeedControllerGroup LeftMotors{leftFront, leftRear};
  // frc::SpeedControllerGroup RightMotors{rightFront, rightRear};

  std::unique_ptr<frc::DifferentialDrive> m_drive;
  frc::DifferentialDriveOdometry m_odometry;
};
