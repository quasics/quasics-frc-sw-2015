// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pneumatics.h"

#include "Constants.h"

Pneumatics::Pneumatics() {
  SetName("Pneumatics");
  SetSubsystem("Pneumatics");
  std::cout << "---------------------------------\n"
            << "Configuring PCM with CAN ID " << CANBusIds::Other::PCM << "\n"
            << "---------------------------------\n";
  RetractSolenoid();
}

void Pneumatics::Periodic() {
  m_compressor.SetClosedLoopControl(compressorEnabled);
}

void Pneumatics::ExtendSolenoid() {
  SetIntakeSolenoidState(frc::DoubleSolenoid::Value::kForward);
}

void Pneumatics::RetractSolenoid() {
  SetIntakeSolenoidState(frc::DoubleSolenoid::Value::kReverse);
}

void Pneumatics::StopSolenoid() {
  SetIntakeSolenoidState(frc::DoubleSolenoid::Value::kOff);
}

void Pneumatics::ToggleSolenoid() {
  m_intakeSolenoid.Toggle();
#ifdef USE_TWO_SOLENOIDS_FOR_INTAKE
  m_intakeSolenoid2.Toggle();
#endif  // USE_TWO_SOLENOIDS_FOR_INTAKE
}

void Pneumatics::SetIntakeSolenoidState(frc::DoubleSolenoid::Value value) {
  m_intakeSolenoid.Set(value);
#ifdef USE_TWO_SOLENOIDS_FOR_INTAKE
  m_intakeSolenoid2.Set(value);
#endif  // USE_TWO_SOLENOIDS_FOR_INTAKE
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
