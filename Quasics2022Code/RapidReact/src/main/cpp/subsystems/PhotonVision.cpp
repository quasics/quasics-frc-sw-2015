// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonVision.h"

PhotonVision::PhotonVision() = default;

// This method will be called once per scheduler run
void PhotonVision::Periodic() {
}

void PhotonVision::GetData() {
  currentData = camera.GetLatestResult();
}

bool PhotonVision::HasTargets() {
  return currentData.HasTargets();
}

photonlib::PhotonTrackedTarget PhotonVision::GetTarget() {
  return currentData.GetBestTarget();
}
