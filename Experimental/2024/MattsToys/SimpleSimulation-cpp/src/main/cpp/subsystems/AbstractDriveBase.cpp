// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AbstractDriveBase.h"

#include <cmath>

AbstractDriveBase::AbstractDriveBase() = default;

// This method will be called once per scheduler run
void AbstractDriveBase::Periodic() {}

void AbstractDriveBase::tankDrive(double leftInputPercent, double rightInputPercent) {
    // Make sure that speeds are limited to a range of -1 to +1
    double leftPercent = std::max(std::min(1.0, leftInputPercent), -1.0);
    double rightPercent = std::max(std::min(1.0, rightInputPercent), -1.0);

    // Other adjustments could happen here (e.g., turbo/turtle mode, or deadband, etc.)

    // Apply the speeds to the motors!
    setMotorSpeeds(leftPercent, rightPercent);
}