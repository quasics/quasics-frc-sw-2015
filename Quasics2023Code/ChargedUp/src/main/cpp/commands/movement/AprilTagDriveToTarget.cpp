// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/AprilTagDriveToTarget.h"

#include <iostream>

AprilTagDriveToTarget::AprilTagDriveToTarget(PhotonLibVision* photonLibVision,
                                             Drivebase* drivebase,
                                             int targetToDriveTo)
    : m_photonLibVision(photonLibVision),
      m_drivebase(drivebase),
      m_targetToDriveTo(targetToDriveTo) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements({photonLibVision, drivebase});
  SetName("AprilTagDriveToTarget");
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
  if (std::abs(m_distance.value()) <
          (PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS
               .value()) &&
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
        std::abs(m_distance.value()),
        PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS
            .value());
    // adding extra speed

    if (forwardSpeed > 0) {
      forwardSpeed = forwardSpeed + 0.2;
    } else {
      forwardSpeed = forwardSpeed - 0.2;
    }
    // deleted a negative sign in rotationSpeed
    double rotationSpeed = turnController.Calculate(m_angle.value(), 0);
    std::cout << "Distance to Target: " << std::abs(m_distance.value())
              << std::endl;

    std::cout << "Angle to Target: " << m_angle.value() << std::endl;
    std::cout << "Forward Speed: " << forwardSpeed
              << " Rotation Speed: " << rotationSpeed << std::endl;
    // m_drivebase->ArcadeDrive(forwardSpeed, rotationSpeed);
  }
}
