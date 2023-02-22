// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonLibVision.h"

PhotonLibVision::PhotonLibVision() = default;

// This method will be called once per scheduler run
void PhotonLibVision::Periodic() {}

bool PhotonLibVision::AprilTagIdentified(double IDWantedTarget) {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    std::span<const photonlib::PhotonTrackedTarget> targets =
        result.GetTargets();
    for (const photonlib::PhotonTrackedTarget& target : targets) {
        }
    // for (i = 0, i++, targets.size()) int targetID = target.GetFiducialId();
  }

  // TODO: Replace this.
  return false;
}
