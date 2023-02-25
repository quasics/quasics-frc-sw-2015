// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonLibVision.h"

PhotonLibVision::PhotonLibVision() = default;

// This method will be called once per scheduler run
void PhotonLibVision::Periodic() {
}

bool PhotonLibVision::AprilTagTargetIdentified(int IDWantedTarget) {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    std::span<const photonlib::PhotonTrackedTarget> targets =
        result.GetTargets();
    for (const photonlib::PhotonTrackedTarget& target : targets) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
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

bool PhotonLibVision::CalculateDistanceAndAngleToTarget(
    int idWantedTarget, units::meter_t& distance, units::degree_t& angle) {
  auto possibleTarget = GetIdentifiedAprilTarget(idWantedTarget);
  if (!possibleTarget.has_value()) {
    return false;
  }

  auto target = possibleTarget.value();

  const units::degree_t angleToTarget(target.GetYaw());
  units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      PhotonVisionConstants::CameraAndTargetValues::CAMERA_HEIGHT,
      PhotonVisionConstants::CameraAndTargetValues::TARGET_HEIGHT,
      PhotonVisionConstants::CameraAndTargetValues::CAMERA_PITCH,
      angleToTarget);

  distance = range;
  angle = angleToTarget;
  return true;
}