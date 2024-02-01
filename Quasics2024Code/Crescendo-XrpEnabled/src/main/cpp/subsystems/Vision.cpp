// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() {
  SetName("Vision");
}

// This method will be called once per scheduler run
void Vision::Periodic() {
}

bool Vision::AprilTagTargetIdentified(int IDWantedTarget) {
  photon::PhotonPipelineResult ID = camera.GetLatestResult();
  if (ID.HasTargets()) {
    std::span<const photon::PhotonTrackedTarget> targets = ID.GetTargets();
    for (const photon::PhotonTrackedTarget target : targets) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
        return true;
      }
    }
  }
  return false;
}

std::optional<photon::PhotonTrackedTarget> Vision::GetIdentifiedAprilTarget(
    int IDWantedTarget) {
  photon::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    for (const photon::PhotonTrackedTarget& target : result.GetTargets()) {
      int targetID = target.GetFiducialId();
      if (targetID == IDWantedTarget) {
        return target;
      }
    }
  }
  return std::nullopt;
}

bool Vision::CalculateDistanceAndAnglesToTarger(int IDWantedTarget,
                                                units::meter_t& distance,
                                                units::degree_t& pitchTarget,
                                                units::degree_t& yawTarget) {
  auto possibleTarget = GetIdentifiedAprilTarget(IDWantedTarget);
  if (!possibleTarget.has_value()) {
    return false;
  }

  auto target = possibleTarget.value();
  const units::degree_t pitchToTarget(target.GetPitch());
  const units::degree_t yawToTarget(target.GetYaw());
  units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      1_in, 1_in, 0_rad, pitchToTarget);

  distance = range;
  pitchTarget = pitchToTarget;
  yawTarget = yawToTarget;
  return true;
}

std::optional<photon::EstimatedRobotPose> Vision::UpdateFieldPosition(
    frc::Pose2d estimatedPose) {
  estimator.SetReferencePose(frc::Pose3d(estimatedPose));
  return estimator.Update();
};