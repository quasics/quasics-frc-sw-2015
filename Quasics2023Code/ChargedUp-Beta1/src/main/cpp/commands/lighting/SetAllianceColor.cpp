// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/lighting/SetAllianceColor.h"

#include <frc/DriverStation.h>

SetAllianceColor::SetAllianceColor(Lighting* lighting) {
  m_lighting = lighting;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
  SetName("SetAllianceColor");
}

// Called when the command is initially scheduled.
void SetAllianceColor::Initialize() {
  m_lighting->SetAllToColor(0, 0, 0);
}

// Called repeatedly when this Command is scheduled to run
void SetAllianceColor::Execute() {
  const auto allianceData = frc::DriverStation::GetAlliance();
  if (allianceData == frc::DriverStation::kRed) {
    m_lighting->SetAllToColor(255, 0, 0);
  } else if (allianceData == frc::DriverStation::kBlue) {
    m_lighting->SetAllToColor(0, 0, 255);
  } else {
    m_lighting->SetAllToColor(0, 255, 0);
  }
}

// Called once the command ends or is interrupted.
void SetAllianceColor::End(bool interrupted) {
}

// Returns true when the command should end.
bool SetAllianceColor::IsFinished() {
  return false;
}
