// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/UltrasonicSensor.h"

#include <frc/smartdashboard/SmartDashboard.h>

UltrasonicSensor::UltrasonicSensor() = default;

// This method will be called once per scheduler run
void UltrasonicSensor::Periodic() {
    frc::SmartDashboard::PutNumber("Distance", m_sensor.GetValue());
}
