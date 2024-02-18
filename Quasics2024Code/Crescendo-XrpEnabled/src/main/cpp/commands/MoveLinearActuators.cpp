// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveLinearActuators.h"

MoveLinearActuators::MoveLinearActuators(LinearActuators& linearActuators,
                                         bool extending)
    : m_linearActuators(linearActuators), m_extending(extending) {
  AddRequirements(&m_linearActuators);
}

// Called when the command is initially scheduled.
void MoveLinearActuators::Initialize() {
  if (m_extending) {
    m_linearActuators.ExtendLinearActuators();
  } else {
    m_linearActuators.RetractLinearActuators();
  }
}

// Returns true when the command should end.
bool MoveLinearActuators::IsFinished() {
  return false;
}
