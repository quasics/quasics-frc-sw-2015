// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pneumatics.h"
#include <iostream>
#include <Constants.h>

Pneumatics::Pneumatics()
    : c(CANBusIds::PneumaticsIds::Compressor),
      IntakeSolenoid(CANBusIds::PneumaticsIds::IntakeSolenoidForward,
                     CANBusIds::PneumaticsIds::IntakeSolenoidBackward) {
}

void Pneumatics::Periodic() {

}

void Pneumatics::ExtendSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Pneumatics::RetractSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Pneumatics::StopSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
}

void Pneumatics::StartCompressor(){
  c.SetClosedLoopControl(true);
 
}

void Pneumatics::StopCompressor(){
 
  c.SetClosedLoopControl(false);
}

bool Pneumatics:: IsCompressorEnabled(){
  return c.Enabled();
}

bool Pneumatics:: GetPressureSwitchValue(){
  return c.GetPressureSwitchValue();
}

double Pneumatics:: GetCompressorCurrent(){
  return c.GetCompressorCurrent();
}
