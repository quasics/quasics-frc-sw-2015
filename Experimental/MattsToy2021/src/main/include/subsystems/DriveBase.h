// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

// #include "../../../../Common2021/CommonDriveSubsystem.h"

class DriveBase : public frc2::SubsystemBase {
 public:
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Stop() {
    drive.StopMotor();
  }

  void ArcadeDrive(double xaxisSpeed, double zaxisRotate, double squareInputs);

  void TankDrive(double leftSpeed, double rightSpeed);

  void ResetEncoders();
  int GetLeftEncoderCount();
  int GetRightEncoderCount();

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

  rev::CANEncoder leftFrontEncoder =
      leftFront.GetEncoder();  //.GetPosition and .GetVelocity
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();
};
