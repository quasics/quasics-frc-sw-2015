// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PatrioticLightsCmd.h"

PatrioticLightsCmd::PatrioticLightsCmd(Lighting* lighting)
    : m_lighting(lighting), m_lastStartingPosition(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(lighting);
}

// Called when the command is initially scheduled.
void PatrioticLightsCmd::Initialize() {
  m_lighting->SetStripColors([this](int pos) {
    const int totalLights = m_lighting->GetNumberOfLEDs();
    const int slotsForColor = totalLights / 3;
    if (pos <= slotsForColor) {
      return frc::AddressableLED::LEDData(0, 0, 255);
    } else if (pos <= (slotsForColor * 2)) {
      return frc::AddressableLED::LEDData(255, 255, 255);
    } else {
      return frc::AddressableLED::LEDData(255, 0, 0);
    }
  });
}

// Called repeatedly when this Command is scheduled to run
void PatrioticLightsCmd::Execute() {
}

// Called once the command ends or is interrupted.
void PatrioticLightsCmd::End(bool interrupted) {
}
