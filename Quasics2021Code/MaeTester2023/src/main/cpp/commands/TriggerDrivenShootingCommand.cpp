// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TriggerDrivenShootingCommand.h"

TriggerDrivenShootingCommand::TriggerDrivenShootingCommand(
    Shooter* shooter, double highSpeed, double lowSpeed,
    std::function<bool()> runHighSpeed, std::function<bool()> runLowSpeed)
    : shooter(shooter),
      highSpeed(highSpeed),
      lowSpeed(lowSpeed),
      runHighSpeed(runHighSpeed),
      runLowSpeed(runLowSpeed) {
  AddRequirements({shooter});
}

// Called repeatedly when this Command is scheduled to run
void TriggerDrivenShootingCommand::Execute() {
  if (runHighSpeed()) {
    shooter->SetSpeed(highSpeed);
  } else if (runLowSpeed()) {
    shooter->SetSpeed(lowSpeed);
  } else {
    shooter->Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerDrivenShootingCommand::End(bool interrupted) {
  shooter->Stop();
}
