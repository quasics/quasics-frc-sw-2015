// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCubeOrConeIntakeSpeed.h"

SetCubeOrConeIntakeSpeed::SetCubeOrConeIntakeSpeed(ConfigSettings* settings)
    : m_settings(settings) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetCubeOrConeIntakeSpeed::Initialize() {
  bool intakingCubeStatus = m_settings->intakingCubes;
  m_settings->intakingCubes = !intakingCubeStatus;
}

// Returns true when the command should end.
bool SetCubeOrConeIntakeSpeed::IsFinished() {
  return true;
}
