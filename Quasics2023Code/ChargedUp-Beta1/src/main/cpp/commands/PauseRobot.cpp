// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PauseRobot.h"

PauseRobot::PauseRobot(Drivebase* drivebase, units::second_t time)
    : m_drivebase(drivebase), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void PauseRobot::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_drivebase->Stop();
}

// Called repeatedly when this Command is scheduled to run
void PauseRobot::Execute() {
  m_drivebase->Stop();
}

// Called once the command ends or is interrupted.
void PauseRobot::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool PauseRobot::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
