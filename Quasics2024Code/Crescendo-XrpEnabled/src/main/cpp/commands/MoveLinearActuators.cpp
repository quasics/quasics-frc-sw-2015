// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveLinearActuators.h"

#include <iostream>

MoveLinearActuators::MoveLinearActuators(LinearActuators& linearActuators,
                                         bool extending)
    : m_linearActuators(linearActuators), m_extending(extending) {
  AddRequirements(&m_linearActuators);
}

// Called when the command is initially scheduled.
void MoveLinearActuators::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  if (m_extending) {
    std::cout << "Starting to extend" << std::endl;
    m_linearActuators.ExtendLinearActuators();
  } else {
    std::cout << "Starting to retract" << std::endl;
    m_linearActuators.RetractLinearActuators();
  }
}

bool MoveLinearActuators::IsFinished() {
  if (m_stopWatch.HasElapsed(4.3_s)) {
    std::cout << "Finished" << std::endl;
    return true;
  }
  return false;
}
