// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/Spark.h>

class MotorOne : public frc2::SubsystemBase {
 public:
  MotorOne();
  void SetSpeed(double percent) { m_motor.Set(percent); }
  void Off() { SetSpeed(0); }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  
 
 public:

  

  frc::Spark m_motor{0};

};
