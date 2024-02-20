// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MatchPlayLighting.h"

#include <frc/DriverStation.h>

MatchPlayLighting::MatchPlayLighting(Lighting* lighting) {
  m_lighting = lighting;
  AddRequirements(m_lighting);
  SetName("MatchPlayLighting");
}

// Called repeatedly when this Command is scheduled to run
void MatchPlayLighting::Execute() {
  const auto allianceData = frc::DriverStation::GetAlliance();

  frc::AddressableLED::LEDData allianceColor = Lighting::GREEN;
  if (allianceData == frc::DriverStation::kRed) {
    allianceColor = Lighting::RED;
  } else if (allianceData == frc::DriverStation::kBlue) {
    allianceColor = Lighting::BLUE;
  }

  // m_lighting->SetLightColors(allianceColor);
}

// Called once the command ends or is interrupted.
void MatchPlayLighting::End(bool interrupted) {
  // m_lighting->setAllToColor(0, 0, 0);
}
