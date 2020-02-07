/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RaiseElevatorCommand.h"

RaiseElevatorCommand::RaiseElevatorCommand(SwissArmySubsystem* targetSubsystem)
    : targetSubsystem(targetSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(targetSubsystem);
}

// Called when the command is initially scheduled.
void RaiseElevatorCommand::Initialize() {
  targetSubsystem->ClimbUp();
}

// Called once the command ends or is interrupted.
void RaiseElevatorCommand::End(bool interrupted) {
  targetSubsystem->StopClimbing();
}
