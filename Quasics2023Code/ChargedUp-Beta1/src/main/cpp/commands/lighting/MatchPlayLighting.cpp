// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/lighting/MatchPlayLighting.h"

#include <frc/DriverStation.h>

#include <iostream>

MatchPlayLighting::MatchPlayLighting(Lighting* lighting,
                                     ConfigSettings* configSettings) {
  m_lighting = lighting;
  m_configSettings = configSettings;
  AddRequirements(m_lighting);
  SetName("MatchPlayLighting");
}

// This is a "normal" member function, which means that it must be called via an
// object (e.g., "aCommand.ColorFunction(4)", or "cmdPtr->ColorFunction(6)").
// As a result, it can't be passed directly to the "SetLightsColors" function in
// the code below; we'll instead need to use "std::bind" to set things up so
// that it has a suitable wrapper that lets it be used like a "raw" function.
frc::AddressableLED::LEDData MatchPlayLighting::GetComplexColorFunction(
    frc::AddressableLED::LEDData allianceColor,
    RequestedPayload requestedPayload, int pos) {
  // Even-numbered LEDs are always the alliance color.
  if (pos % 2 == 0) {
    return allianceColor;
  }

  if (requestedPayload == RequestedPayload::eNothing) {
    return allianceColor;
  }
  if (requestedPayload == RequestedPayload::eCones) {
    return Lighting::ORANGE;
  } else if (requestedPayload == RequestedPayload::eCubes) {
    return Lighting::GREEN;
  }

  // TODO: Add logic to return the right color for odd-numbered LEDs.  For now,
  // we'll just use the alliance color for everything.
  return Lighting::WHITE;
  ;
}

frc::AddressableLED::LEDData MatchPlayLighting::SimpleColorFunction(
    int position) {
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
  const bool normalDriveEngaged = m_configSettings->normalDriveEngaged;

  frc::AddressableLED::LEDData allianceColor = Lighting::GREEN;
  if (normalDriveEngaged) {
    if (allianceData == frc::DriverStation::kRed) {
      allianceColor = Lighting::RED;
    } else if (allianceData == frc::DriverStation::kBlue) {
      allianceColor = Lighting::BLUE;
    }
  } else {
    if (allianceData == frc::DriverStation::kRed) {
      allianceColor = Lighting::PINK;
    } else if (allianceData == frc::DriverStation::kBlue) {
      allianceColor = Lighting::CYAN;
    }
  }

  // Set the lights, based on the above information.
  m_lighting->SetLightColors(std::bind(
      // This is the member function we want to have invoked
      &MatchPlayLighting::GetComplexColorFunction,
      // It (invisibly) has "this" passed as its first parameter, providing
      // the object on which the function was called.
      this,
      // We want to pass in the alliance color as the next parameter
      allianceColor,
      // We want to pass in the requested payload as the next parameter
      requestedPayload,
      // There's another paramter that will be provided when the function is
      // *used*, so we provide a "placeholder" for the bind() function.
      std::placeholders::_1));
}

// Called once the command ends or is interrupted.
void MatchPlayLighting::End(bool interrupted) {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Returns true when the command should end.
bool MatchPlayLighting::IsFinished() {
  return false;
}
