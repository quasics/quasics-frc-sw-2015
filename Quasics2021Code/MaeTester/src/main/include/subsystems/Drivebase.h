// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Stop();
  void setMotorSpeed(double leftSpeed, double rightSpeed);
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;
};
