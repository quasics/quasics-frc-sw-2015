// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "../../../../Common2021/CommonDriveSubsystem.h"

// TODO(mjh): Decide if we want to just use values from 1 encoder per side for
// position/distance, or take an average, or what.
class DriveBase : public CommonDriveSubsystem {
 public:
  DriveBase();

  //
  // Methods defined in frc2::SubsystemBase
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //
  // Methods defined in CommonDriveSubsystem.
 public:
  void Stop() override {
    drive.StopMotor();
  }

  void ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                   bool squareInputs) override;

  void TankDrive(double leftSpeed, double rightSpeed) override;

  void ResetEncoders() override;
  double GetLeftEncoderCount() override;
  double GetRightEncoderCount() override;
  units::meter_t GetRightDistance() override;
  units::meter_t GetLeftDistance() override;

  frc::Gyro& GetZAxisGyro() override {
    return adiGyro;
  }

  //
  // Methods added in this class.
 public:
  void SetCoastingEnabled(bool enabled);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

  frc::SpeedControllerGroup leftMotors{leftFront, leftRear};
  frc::SpeedControllerGroup rightMotors{rightFront, rightRear};

  frc::DifferentialDrive drive{leftMotors, rightMotors};

  rev::CANEncoder leftFrontEncoder = leftFront.GetEncoder();
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();

  frc::ADXRS450_Gyro adiGyro{
      frc::SPI::Port::kOnboardCS0};  // Chip Select jumper is set to CS0
};
