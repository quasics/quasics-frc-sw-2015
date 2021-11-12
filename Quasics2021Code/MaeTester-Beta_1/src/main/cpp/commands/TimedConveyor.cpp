// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TimedConveyor.h"

TimedConveyor::TimedConveyor(Intake* intake, units::second_t time, bool forward)
    : intake(intake), time(time), forward(forward) {
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void TimedConveyor::Initialize() {
  if (forward) {
    intake->ConveyBallOn();
  }  else  {
    intake->ConveyBallReverse();
  }

  // [Re]start the timer
  stopWatch.Reset();
  stopWatch.Start();
}

// Called repeatedly when this Command is scheduled to run
void TimedConveyor::Execute() {
  // Be paranoid, just in case the CAN bus wants to keep being told...
  if (forward) {
    intake->ConveyBallOn();
  } else {
    intake->ConveyBallReverse();
  }
}

// Called once the command ends or is interrupted.
void TimedConveyor::End(bool interrupted) {
  intake->ConveyBallOff();
}

// Returns true when the command should end.
bool TimedConveyor::IsFinished() {
  return stopWatch.HasElapsed(time);
}
