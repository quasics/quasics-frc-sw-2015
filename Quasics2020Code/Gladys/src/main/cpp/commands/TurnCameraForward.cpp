/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnCameraForward.h"

TurnCameraForward::TurnCameraForward(CameraStand * cameraStand) : cameraStand(cameraStand) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(cameraStand);
}

// Called when the command is initially scheduled.
void TurnCameraForward::Initialize() {
  cameraStand->TurnCameraMin();
}

// Called repeatedly when this Command is scheduled to run

// Called once the command ends or is interrupted.
void TurnCameraForward::End(bool interrupted) {}

// Returns true when the command should end.
bool TurnCameraForward::IsFinished() { return true; }
