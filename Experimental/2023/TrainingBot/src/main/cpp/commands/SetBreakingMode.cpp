// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetBreakingMode.h"

SetBreakingMode::SetBreakingMode(DriveBase* driveBase, bool enableBreaking) {
  m_driveBase = driveBase;
  m_enableBreaking = enableBreaking;

  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void SetBreakingMode::Initialize() {
  m_driveBase->EnableBreakingMode(m_enableBreaking);
}

// Returns true when the command should end.
bool SetBreakingMode::IsFinished() {
  return true;
}
