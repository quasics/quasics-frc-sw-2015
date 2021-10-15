// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DelayForTime.h"

DelayForTime::DelayForTime(units::second_t time) : time(time) {
  // There are no subsystem dependencies for this command.
}

// Called when the command is initially scheduled.
void DelayForTime::Initialize() {
  stopWatch.Reset();
  stopWatch.Start();
}

// Returns true when the command should end.
bool DelayForTime::IsFinished() {
  return stopWatch.HasElapsed(time);
}
