 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnCameraBackward.h"

TurnCameraBackward::TurnCameraBackward(CameraStand * cameraStand): cameraStand(cameraStand) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(cameraStand);
  }

// Called when the command is initially scheduled.
void TurnCameraBackward::Initialize() {
  cameraStand->TurnCameraMax();
}

// Called repeatedly when this Command is scheduled to run

// Called once the command ends or is interrupted.
void TurnCameraBackward::End(bool interrupted) {}

// Returns true when the command should end.
bool TurnCameraBackward::IsFinished() { return true; }
