// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

class Pneumatics : public frc2::SubsystemBase {
 public:
  Pneumatics();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 public:
  // Solenoid control
  void ExtendSolenoid();
  void RetractSolenoid();
  void StopSolenoid();

 public:
  // Compressor control

  /**
   * Sets whether the compressor should automatically turn on when pressure is
   * low.
   *
   * Note: will take effect on the next invocation of "Periodic()".
   */
  void SetCompressorEnabled(bool tf);

  /** Returns true iff the compressor is currently running (output is active).
   */
  bool IsCompressorRunning();

  /** Returns true iff the pressure switch is triggered. */
  bool GetPressureSwitchValue();

  /** Returns how much current the compressor is drawing. */
  double GetCompressorCurrent();

 private:
  frc::Compressor m_compressor;
  frc::DoubleSolenoid m_intakeSolenoid;
  bool compressorEnabled = true;
};
