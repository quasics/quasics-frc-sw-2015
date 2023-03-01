// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleBrakingMode.h"

ToggleBrakingMode::ToggleBrakingMode(Drivebase* drivebase)
    : m_drivebase(drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  // NOTE WE ARE KNOWINGLY NOT ADDING REQUIREMENTS HERE THIS IS SO WE CAN CHANGE
  // THE BRAKING MODE WITHOUT STOPPING TO DRIVE
}

// Called when the command is initially scheduled.
void ToggleBrakingMode::Initialize() {
  bool status = m_drivebase->IsInBrakingMode();
  m_drivebase->SetBrakingMode(!status);
}

// Returns true when the command should end.
bool ToggleBrakingMode::IsFinished() {
  return true;
}
