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

bool Vision::AprilTagTargetFound(int TargetID) {
  photon::PhotonPipelineResult ID = camera.GetLatestResult();
  return false;
}
