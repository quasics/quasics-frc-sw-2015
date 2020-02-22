/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DeliverForTime.h"

DeliverForTime::DeliverForTime(Exhaust* exhaust, double duration)
    : exhaust(exhaust), duration(duration) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(exhaust);
}

// Called when the command is initially scheduled.
void DeliverForTime::Initialize() {
  tiktok.Start();
  tiktok.Reset();
  exhaust->PushBallDown();
  exhaust->ShootBallOn();
}

// Called once the command ends or is interrupted.
void DeliverForTime::End(bool interrupted) {
  exhaust->PushBallOff();
  exhaust->ShootBallOff();
}

// Returns true when the command should end.
bool DeliverForTime::IsFinished() {
  return tiktok.HasPeriodPassed(units::second_t(duration));
}
