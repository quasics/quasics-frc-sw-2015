// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

// #define USE_TWO_SOLENOIDS_FOR_INTAKE

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
  void ToggleSolenoid();

 private:
  /**
   * Utility method, used to (mostly) encapsulate knowledge of how many
   * solenoids are being used for the intake.
   */
  void SetIntakeSolenoidState(frc::DoubleSolenoid::Value value);

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
   // Compressor control
   frc::Compressor m_compressor{PneumaticsIds::DefaultSolenoidId};
   bool compressorEnabled = true;

   // Solenoid(s)
   frc::DoubleSolenoid m_intakeSolenoid{CANBusIds::Other::PCM,
                                        PneumaticsIds::IntakeSolenoidForward,
                                        PneumaticsIds::IntakeSolenoidBackward};
#ifdef USE_TWO_SOLENOIDS_FOR_INTAKE
   frc::DoubleSolenoid m_intakeSolenoid2{
       CANBusIds::Other::PCM, PneumaticsIds::IntakeSolenoid2Forward,
       PneumaticsIds::IntakeSolenoid2Backward};
#endif  // USE_TWO_SOLENOIDS_FOR_INTAKE
};
