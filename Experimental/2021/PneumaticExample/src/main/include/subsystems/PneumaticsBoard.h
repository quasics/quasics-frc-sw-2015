// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class PneumaticsBoard : public frc2::SubsystemBase {
 public:
  PneumaticsBoard() {
    SetName("PneumaticsBoard");
    SetSubsystem("PneumaticsBoard");
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {
    // Ensures that the PCM is periodically "tickled".
    m_compressor.SetClosedLoopControl(compressorEnabled);
  }

 public:
  // Solenoid control
  void ExtendSolenoid() {
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  void RetractSolenoid() {
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  void ToggleSolenoid() {
    m_intakeSolenoid.Toggle();
  };
  void StopSolenoid() {
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  }

 public:
  // Compressor control

  /**
   * Sets whether the compressor should automatically turn on when pressure is
   * low.
   *
   * Note: will take effect on the next invocation of "Periodic()".
   */
  void SetCompressorEnabled(bool tf) {
    compressorEnabled = tf;
  }

  /** Returns true iff the compressor is currently running (output is active).
   */
  bool IsCompressorRunning() {
    return m_compressor.Enabled();
  }

  /** Returns true iff the pressure switch is triggered. */
  bool GetPressureSwitchValue() {
    return m_compressor.GetPressureSwitchValue();
  }

  /** Returns how much current the compressor is drawing. */
  double GetCompressorCurrent() {
    return m_compressor.GetCompressorCurrent();
  }

 private:
  frc::Compressor m_compressor{PneumaticIds::Compressor};
  frc::DoubleSolenoid m_intakeSolenoid{CANBusIds::PCM,
                                       PneumaticIds::IntakeSolenoidForward,
                                       PneumaticIds::IntakeSolenoidBackward};
  bool compressorEnabled = true;
};
