// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootForTime.h"

ShootForTime::ShootForTime(Shooter *shooter, units::second_t time, double speed)
    : shooter(shooter), time(time), speed(speed)
{
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ShootForTime::Initialize()
{
  shooter->SetSpeed(speed);

  stopWatch.Reset();
  stopWatch.Start();
}

// Called repeatedly when this Command is scheduled to run
void ShootForTime::Execute()
{
  shooter->SetSpeed(speed);
}

// Called once the command ends or is interrupted.
void ShootForTime::End(bool interrupted)
{
  shooter->Stop();
}

// Returns true when the command should end.
bool ShootForTime::IsFinished()
{
  return stopWatch.HasElapsed(time);
}
