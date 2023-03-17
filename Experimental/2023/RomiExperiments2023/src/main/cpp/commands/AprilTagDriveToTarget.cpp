// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagDriveToTarget.h"
#include <iostream>

AprilTagDriveToTarget::AprilTagDriveToTarget(PhotonLibVision* photonLibVision,
                                             Drivetrain* drivetrain,
                                             int targetToDriveTo)
    : m_photonLibVision(photonLibVision),
      m_drivetrain(drivetrain),
      m_targetToDriveTo(targetToDriveTo) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements({photonLibVision, drivetrain});
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
  m_drivetrain->Stop();
}

// Returns true when the command should end.
bool AprilTagDriveToTarget::IsFinished() {
  if (m_distance <
          (PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS +
           2_in) &&
      ((m_angle > -2_deg) && (m_angle < 2_deg))) {
    std::cout << "Destination Reached" << std::endl;
    return true;
  }
  return false;
}

void AprilTagDriveToTarget::UpdateDrivingParameters() {
  std::cout << "Beginning to Look at Stuff" << std::endl;
  bool targetFound =
      m_photonLibVision->AprilTagTargetIdentified(m_targetToDriveTo);
  if (targetFound == true) {
    m_photonLibVision->CalculateDistanceAndAngleToTarget(m_targetToDriveTo,
                                                         m_distance, m_angle);
    std::cout << "Performing Calculations based on distance"<< m_distance.value() << " and angle "<< m_angle.value() << std::endl;
    double forwardSpeed = -forwardController.Calculate(
        m_distance.value(),
        PhotonVisionConstants::CameraAndTargetValues::GOAL_RANGE_METERS
            .value());
    double rotationSpeed = turnController.Calculate(m_angle.value(), 0);
    std::cout << "Sending Calculations: Speed: "<< forwardSpeed << " Rotation: "<< rotationSpeed << std::endl;
    m_drivetrain->ArcadeDrive(forwardSpeed, rotationSpeed);
  }
}
