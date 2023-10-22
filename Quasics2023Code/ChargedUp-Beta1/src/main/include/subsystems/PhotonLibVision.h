// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#ifdef ENABLE_PHOTONLIB
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#endif  // ENABLE_PHOTONLIB

#include <optional>

#include "Constants.h"

// MAKE SURE LIBRARY HAS THIS FILE INSTALLED
// https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

/**
 * TODO: Add comments describing the class as a whole.
 */
// CODE_REVIEW(matthew): This class should have a comment block (above)
// describing what it is/does.
class PhotonLibVision : public frc2::SubsystemBase {
 public:
  PhotonLibVision();

#ifdef ENABLE_PHOTONLIB
  bool AprilTagTargetIdentified(int IDWantedTarget);

  std::optional<photonlib::PhotonTrackedTarget> GetIdentifiedAprilTarget(
      int IDWantedTarget);

  bool GetIdentifiedAprilTarget(int IDWantedTarget,
                                photonlib::PhotonTrackedTarget &result);

  bool CalculateDistanceAndAngleToTarget(int idWantedTarget,
                                         units::meter_t &distance,
                                         units::degree_t &angle);
#endif  // ENABLE_PHOTONLIB

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
#ifdef ENABLE_PHOTONLIB
  photonlib::PhotonCamera camera{"USB_Camera"};
#endif  // ENABLE_PHOTONLIB
};
