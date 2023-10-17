// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonLibVision.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonCamera.h>
// #include <photonlib/RobotPoseEstimator.h>

#include <iostream>

#include "Constants.h"
#include "math.h"

#undef PITCH_CALIBRATION

PhotonLibVision::PhotonLibVision() {
  SetName("PhotonLibVision");
  frc::SmartDashboard::PutNumber("Length to april tag", 0);
  frc::SmartDashboard::PutNumber("Target id", 0);
}

// This method will be called once per scheduler run
void PhotonLibVision::Periodic() {
#ifdef PITCH_CALIBRATION
  double length = frc::SmartDashboard::GetNumber("Length to april tag", 0);
  int idWantedTarget = frc::SmartDashboard::GetNumber("Target id", 0);
  // std::cerr << "length to target " << length << std::endl;

  units::meter_t aprilTagHeight =
      PhotonVisionConstants::CameraAndTargetValues::TARGET_HEIGHT;
  units::meter_t cameraHeight =
      PhotonVisionConstants::CameraAndTargetValues::CAMERA_HEIGHT;

  units::degree_t wantedTargetPitch =
      1_rad * std::atan(((aprilTagHeight - cameraHeight) / length).value()) -

      PhotonVisionConstants::CameraAndTargetValues::CAMERA_PITCH;

  auto possibleTarget = GetIdentifiedAprilTarget(idWantedTarget);
  if (!possibleTarget.has_value()) {
    std::cerr << "No target identified" << std::endl;
    return;
  }
  auto target = possibleTarget.value();

  const units::degree_t pitchToTarget(target.GetPitch());
  frc::SmartDashboard::PutNumber("pitchToTarget", pitchToTarget.value());
  frc::SmartDashboard::PutNumber("wantedTargetPitch",
                                 wantedTargetPitch.value());
#endif
}

bool PhotonLibVision::AprilTagTargetIdentified(int IDWantedTarget) {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    std::span<const photonlib::PhotonTrackedTarget> targets =
        result.GetTargets();
    for (const photonlib::PhotonTrackedTarget& target : targets) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
        // std::cout << "ID " << IDWantedTarget << "Acquired" << std::endl;
        return true;
      }
    }
  }
  return false;
}

std::optional<photonlib::PhotonTrackedTarget>
PhotonLibVision::GetIdentifiedAprilTarget(int IDWantedTarget) {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    for (const photonlib::PhotonTrackedTarget& target : result.GetTargets()) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
        return target;
      }
    }
  }
  return std::nullopt;
}

/*
  auto possibleTrackedTarget =
  plv_sub.GetIdentifiedAprilTarget(desiredTargetNum); if
  (possibleTrackedTarget.has_value()) { auto target = plv_sub.value();
    // Do stuff with the target....
  }
*/

/*
bool PhotonLibVision::GetIdentifiedAprilTarget(
    double IDWantedTarget, photonlib::PhotonTrackedTarget& result) {
  photonlib::PhotonPipelineResult pipeline_result = camera.GetLatestResult();
  if (pipeline_result.HasTargets()) {
    for (const photonlib::PhotonTrackedTarget& target :
         pipeline_result.GetTargets()) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
        result = target;
        return true;
      }
    }
  }

  return false;
}
*/
/*
  photonlib::PhotonTrackedTarget targetResult;
  if (plv_sub.GetIdentifiedAprilTarget(desiredTargetNum, targetResult)) {
    // Do stuff with the target_result....
  }
*/

bool PhotonLibVision::CalculateDistanceAndAnglesToTarget(
    int idWantedTarget, units::meter_t& distance, units::degree_t& pitchTarget,
    units::degree_t& yawTarget) {
  auto possibleTarget = GetIdentifiedAprilTarget(idWantedTarget);
  if (!possibleTarget.has_value()) {
    return false;
  }

  auto target = possibleTarget.value();

  const units::degree_t pitchToTarget(target.GetPitch());
  const units::degree_t yawToTarget(target.GetYaw());
  units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      PhotonVisionConstants::CameraAndTargetValues::CAMERA_HEIGHT,
      PhotonVisionConstants::CameraAndTargetValues::TARGET_HEIGHT,
      PhotonVisionConstants::CameraAndTargetValues::CAMERA_PITCH,
      pitchToTarget);

  distance = range;
  pitchTarget = pitchToTarget;
  yawTarget = yawToTarget;
  return true;
}

bool PhotonLibVision::GetFieldPosition() {
  // Transfered Everything into the h. file

    // THIS STUFF CAN BE SUBSTITUTED WITH THE OFFICIAL APRIL TAG LAYOUT EVENTUALLY

  /*
  std::vector<frc::AprilTag> tags = {
      {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3),
                      frc::Rotation3d())},
      {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5),
                      frc::Rotation3d())}};


  frc::AprilTagFieldLayout aprilTags(tags, 54_ft, 27_ft);
  //DOWN TO HERE




  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(0.5_m, 0_m, 0.5_m),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));


  photonlib::PhotonPoseEstimator estimator(
      aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE,
      photonlib::PhotonCamera("USB_Camera"), robotToCam);

  */
  return true;
}