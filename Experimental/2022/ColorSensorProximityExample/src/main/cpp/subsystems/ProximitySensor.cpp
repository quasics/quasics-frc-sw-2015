// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ProximitySensor.h"

#include <frc/smartdashboard/SmartDashboard.h>

ProximitySensor::ProximitySensor()
{
    
}

// This method will be called once per scheduler run
void ProximitySensor::Periodic() {
    const auto reading = GetProximity();
    std::cerr << "Proximity: " << reading << std::endl;
    frc::SmartDashboard::PutNumber("Proximity", reading);
}
