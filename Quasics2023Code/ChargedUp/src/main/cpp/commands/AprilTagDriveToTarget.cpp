// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagDriveToTarget.h"

AprilTagDriveToTarget::AprilTagDriveToTarget(PhotonLibVision* photonLibVision,
                                             Drivebase* drivebase,
                                             int targetToDriveTo)
    : m_photonLibVision(photonLibVision),
      m_drivebase(drivebase),
      m_targetToDriveTo(targetToDriveTo) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements({photonLibVision, drivebase});
}

// Called when the command is initially scheduled.
void AprilTagDriveToTarget::Initialize() {
  UpdateDrivingParameters();
}

// Called repeatedly when this Command is scheduled to run
void AprilTagDriveToTarget::Execute() {
  UpdateDrivingParameters();
}

// Called once the command ends or is interrupted.
void AprilTagDriveToTarget::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool AprilTagDriveToTarget::IsFinished() {
  if (m_distance <
          (PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS +
           2_in) &&
      ((m_angle > -2_deg) && (m_angle < 2_deg))) {
    return true;
  }
  return false;
}

void AprilTagDriveToTarget::UpdateDrivingParameters() {
  bool targetFound =
      m_photonLibVision->AprilTagTargetIdentified(m_targetToDriveTo);
  if (targetFound == true) {
    m_photonLibVision->CalculateDistanceAndAngleToTarget(m_targetToDriveTo,
                                                         m_distance, m_angle);
    double forwardSpeed = -forwardController.Calculate(
        m_distance.value(),
        PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS
            .value());
    double rotationSpeed = -turnController.Calculate(m_angle.value(), 0);
    m_drivebase->ArcadeDrive(forwardSpeed, rotationSpeed);
  }
}