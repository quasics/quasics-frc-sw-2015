// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Wait.h"

Wait::Wait(units::second_t time) : m_time(time) {
}

// Called when the command is initially scheduled.
void Wait::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
}

// Returns true when the command should end.
bool Wait::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}