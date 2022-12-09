// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonLibVision.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

PhotonLibVision::PhotonLibVision() = default;

// This method will be called once per scheduler run

double CameraYaw(){
    return 1;
}

void PhotonLibVision::Periodic() {
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    const bool hasTargets = result.HasTargets();
    static unsigned int counter = 0;
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    double skew = 0;
    if (hasTargets) {
        //wpi::ArrayRef<photonlib::PhotonTrackedTarget> targets = result.GetTargets();
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        yaw = target.GetYaw();
        pitch = target.GetPitch();
        area = target.GetArea();
        skew = target.GetSkew();
        //frc::Transform2d pose = target.GetCameraToTarget();
        // wpi::SmallVector<std::pair<double, double>, 4> corners = target.GetCorners();
        //int targetID = target.GetFiducialId();
        //double poseAmbiguity = target.GetPoseAmbiguity();
        //frc::Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        //frc::Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    } else {
        // std::cerr << "No targets, not updating dashboard.\n";
    }
    frc::SmartDashboard::PutBoolean("Has Targets", hasTargets);
    frc::SmartDashboard::PutNumber("yaw", yaw);
    frc::SmartDashboard::PutNumber("pitch", pitch);
    frc::SmartDashboard::PutNumber("area", area);
    frc::SmartDashboard::PutNumber("skew", skew);
    frc::SmartDashboard::PutNumber("counter", counter++);
}
