// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

// "stock colors"
const frc::AddressableLED::LEDData Lighting::WHITE{255, 255, 255};
const frc::AddressableLED::LEDData Lighting::BLACK{0, 0, 0};
const frc::AddressableLED::LEDData Lighting::GREEN{0, 255, 0};
const frc::AddressableLED::LEDData Lighting::BLUE{0, 0, 255};
const frc::AddressableLED::LEDData Lighting::RED{255, 0, 0};

Lighting::Lighting() = default;

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
