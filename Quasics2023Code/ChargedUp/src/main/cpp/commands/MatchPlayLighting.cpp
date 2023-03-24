// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MatchPlayLighting.h"

#include <frc/DriverStation.h>

MatchPlayLighting::MatchPlayLighting(Lighting* lighting,
                                     ConfigSettings* configSettings) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_lighting = lighting;
  m_configSettings = configSettings;
  AddRequirements(m_lighting);
}

frc::AddressableLED::LEDData MatchPlayLighting::colorFunction(int position) {
  if (position % 2 == 0) {
    return Lighting::RED;
  } else {
    return Lighting::BLACK;
  }
}

// Called when the command is initially scheduled.
void MatchPlayLighting::Initialize() {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Called repeatedly when this Command is scheduled to run
void MatchPlayLighting::Execute() {
  // Figure out what we need to communicate.
  const auto allianceData = frc::DriverStation::GetAlliance();
  const RequestedPayload requestedPayload = m_configSettings->requestedPayload;
  const bool singleLights = (requestedPayload == RequestedPayload::eNothing);

  // Set the lights, based on the above information.
  if (singleLights) {
    // We're just showing one color (solid).
    if (allianceData == frc::DriverStation::kRed) {
      m_lighting->SetAllToColor(255, 0, 0);
    } else if (allianceData == frc::DriverStation::kBlue) {
      m_lighting->SetAllToColor(0, 0, 255);
    } else {
      m_lighting->SetAllToColor(0, 255, 0);
    }
  } else {
    // We have some sort of pattern we need to show
    frc::AddressableLED::LEDData allianceColor = Lighting::GREEN;
    if (allianceData == frc::DriverStation::kRed) {
      allianceColor = Lighting::RED;
    } else if (allianceData == frc::DriverStation::kBlue) {
      allianceColor = Lighting::BLUE;
    }

    /* frc::AddressableLED::LEDData gamePieceIndicator;
     if (requestedPayload == RequestedPayload::eCones) {
       m_lighting->SetLightColors(colorFunction);
     } else if (requestedPayload == RequestedPayload::eCubes) {
       m_lighting->SetLightColors(colorFunction);
     } */
  }
}

// Called once the command ends or is interrupted.
void MatchPlayLighting::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool MatchPlayLighting::IsFinished() {
  return false;
}
