// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveRobotToAprilTag.h"

MoveRobotToAprilTag::MoveRobotToAprilTag(Drivebase* drivebase,
                                         PhotonVision* photonVision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivebase, photonVision});
  m_drivebase = drivebase;
  m_photonvision = photonVision;
}

// Called when the command is initially scheduled.
void MoveRobotToAprilTag::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void MoveRobotToAprilTag::Execute() {
  if (m_photonvision->HasTargets()) {
    rotationSpeed =
        -controller.Calculate(m_photonvision->GetTarget().GetYaw(), 0);
  } else {
    rotationSpeed = 0;
  }
  m_drivebase->ArcadeDrive(0, rotationSpeed);
}

// Called once the command ends or is interrupted.
void MoveRobotToAprilTag::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool MoveRobotToAprilTag::IsFinished() {
  return false;
}
