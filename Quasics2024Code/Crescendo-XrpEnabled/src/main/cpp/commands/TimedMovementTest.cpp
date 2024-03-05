// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TimedMovementTest.h"

TimedMovementTest::TimedMovementTest(IDrivebase& drivebase, double speed,
                                     units::second_t time, bool forward)
    : m_drivebase(drivebase),
      m_speed(forward ? -std::abs(speed) : std::abs(speed)),
      m_time(time),
      m_forward(forward) {
  AddRequirements(&m_drivebase);
}

// Called when the command is initially scheduled.
void TimedMovementTest::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_drivebase.tankDrive(m_speed, m_speed);
}

// Called repeatedly when this Command is scheduled to run
void TimedMovementTest::Execute() {
  m_drivebase.tankDrive(m_speed, m_speed);
}

// Called once the command ends or is interrupted.
void TimedMovementTest::End(bool interrupted) {
  m_drivebase.stop();
}

// Returns true when the command should end.
bool TimedMovementTest::IsFinished() {
  return m_stopWatch.HasElapsed(m_time);
}
