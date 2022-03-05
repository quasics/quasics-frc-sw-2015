// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunConveyorAtSpeedForTime.h"

RunConveyorAtSpeedForTime::RunConveyorAtSpeedForTime(Conveyor* conveyor,
                                                     double speed,
                                                     units::second_t time)
    : m_conveyor(conveyor), m_conveyorSpeed(speed), m_time(time) {
  AddRequirements(m_conveyor);
}

// Called when the command is initially scheduled.
void RunConveyorAtSpeedForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_conveyor->SetConveyorSpeed(m_conveyorSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunConveyorAtSpeedForTime::Execute() {
  m_conveyor->SetConveyorSpeed(m_conveyorSpeed);
}

// Called once the command ends or is interrupted.
void RunConveyorAtSpeedForTime::End(bool interrupted) {
  m_conveyor->Stop();
}

// Returns true when the command should end.
bool RunConveyorAtSpeedForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
