// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Pnematics.h"
#include <iostream>
#include <Constants.h>

Pnematics::Pnematics()
    : c(CANBusIds::PneumaticsIds::Compressor),
      IntakeSolenoid(CANBusIds::PneumaticsIds::IntakeSolenoidForward,
                     CANBusIds::PneumaticsIds::IntakeSolenoidBackward) {
}

void Pnematics::Periodic() {

}

void Pnematics::ExtendSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Pnematics::RetractSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Pnematics::StopSolenoid() {
  IntakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
}

void Pnematics::StartCompressor(){
  c.SetClosedLoopControl(true);
 
}

void Pnematics::StopCompressor(){
 
  c.SetClosedLoopControl(false);
}

bool Pnematics:: IsCompressorEnabled(){
  return c.Enabled();
}

bool Pnematics:: GetPressureSwitchValue(){
  return c.GetPressureSwitchValue();
}

double Pnematics:: GetCompressorCurrent(){
  return c.GetCompressorCurrent();
}