// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include <optional>

#include "Constants.h"

// MAKE SURE LIBRARY HAS THIS FILE INSTALLED
// https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

class PhotonLibVision : public frc2::SubsystemBase {
 public:
  PhotonLibVision();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool AprilTagTargetIdentified(int IDWantedTarget);

  std::optional<photonlib::PhotonTrackedTarget> GetIdentifiedAprilTarget(
      int IDWantedTarget);
  bool GetIdentifiedAprilTarget(int IDWantedTarget,
                                photonlib::PhotonTrackedTarget& result);

  bool CalculateDistanceAndAngleToTarget(int idWantedTarget,
                                         units::meter_t& distance,
                                         units::degree_t& angle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photonlib::PhotonCamera camera{"USB_Camera"};
};
