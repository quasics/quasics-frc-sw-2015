// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonLibVision.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

PhotonLibVision::PhotonLibVision() = default;

// This method will be called once per scheduler run
void PhotonLibVision::Periodic() {
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    bool hasTargets = result.HasTargets();
    //wpi::ArrayRef<photonlib::PhotonTrackedTarget> targets = result.GetTargets();
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    double yaw = target.GetYaw();
    double pitch = target.GetPitch();
    double area = target.GetArea();
    double skew = target.GetSkew();
    //frc::Transform2d pose = target.GetCameraToTarget();
    wpi::SmallVector<std::pair<double, double>, 4> corners = target.GetCorners();
    //int targetID = target.GetFiducialId();
    //double poseAmbiguity = target.GetPoseAmbiguity();
    //frc::Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    //frc::Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    frc::SmartDashboard::PutBoolean("Has Targets", hasTargets);
    frc::SmartDashboard::PutNumber("yaw", yaw);
    frc::SmartDashboard::PutNumber("pitch", pitch);
    frc::SmartDashboard::PutNumber("area", area);
    frc::SmartDashboard::PutNumber("skew", skew);
}
