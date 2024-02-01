// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetRobotOdometry.h"

SetRobotOdometry::SetRobotOdometry(IDrivebase& driveBase, frc::Pose2d pose)
    : m_driveBase(driveBase), m_pose(pose) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_driveBase);
}

// Called when the command is initially scheduled.
void SetRobotOdometry::Initialize() {
  m_driveBase.resetOdometry(m_pose);
}

// Returns true when the command should end.
bool SetRobotOdometry::IsFinished() {
  return true;
}
