// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pneumatics.h"

#include <Constants.h>

Pneumatics::Pneumatics()
    : m_compressor(PneumaticsIds::Compressor),
      m_intakeSolenoid(CANBusIds::Other::PCM,
                       PneumaticsIds::IntakeSolenoidForward,
                       PneumaticsIds::IntakeSolenoidBackward) {
  SetName("Pneumatics");
  SetSubsystem("Pneumatics");
}

void Pneumatics::Periodic() {
  // Insert any periodic execution code here.
  m_compressor.SetClosedLoopControl(compressorEnabled);
}

void Pneumatics::ExtendSolenoid() {
  m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Pneumatics::RetractSolenoid() {
  m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Pneumatics::StopSolenoid() {
  m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
}

void Pneumatics::ToggleSolenoid() {
  m_intakeSolenoid.Toggle();
}

void Pneumatics::SetCompressorEnabled(bool tf) {
  compressorEnabled = tf;
}

bool Pneumatics::IsCompressorRunning() {
  return m_compressor.Enabled();
}

bool Pneumatics::GetPressureSwitchValue() {
  return m_compressor.GetPressureSwitchValue();
}

double Pneumatics::GetCompressorCurrent() {
  return m_compressor.GetCompressorCurrent();
}
