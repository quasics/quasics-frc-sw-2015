// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

class Pnematics : public frc2::SubsystemBase {
 public:
  Pnematics();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */

  void ExtendSolenoid();
  void RetractSolenoid();
  void StopSolenoid();
  void StartCompressor();
  void StopCompressor();
  bool IsCompressorEnabled();
  bool GetPressureSwitchValue();
  double GetCompressorCurrent();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Compressor c{0};
  frc::DoubleSolenoid IntakeSolenoid{1, 2};

};
