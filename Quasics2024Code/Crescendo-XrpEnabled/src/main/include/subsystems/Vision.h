// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Translation3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>

#include <map>
#include <optional>

#include "Constants.h"

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  bool AprilTagTargetIdentified(int IDWantedTarget);

  std::optional<photon::PhotonTrackedTarget> GetIdentifiedAprilTarget(
      int IDWantedTarget);

  bool GetIdentifiedAprilTarget(int TargetID,
                                photon::PhotonTrackedTarget& result);

  bool CalculateDistanceAndAnglesToTarger(int IDWantedTarget,
                                          units::meter_t& distance,
                                          units::degree_t& pitchTarget,
                                          units::degree_t& yawTarget);

  frc::Translation3d GetTargetPlacement(int targetID);

  std::optional<photon::EstimatedRobotPose> UpdateFieldPosition(
      frc::Pose2d estimatedPose);

  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photon::PhotonCamera camera{"USB_Camera"};

  std::vector<frc::AprilTag> tags = {
      {1, frc::Pose3d(0_in, 0_in, 17_in, frc::Rotation3d())}};

  frc::AprilTagFieldLayout aprilTags =
      frc::AprilTagFieldLayout(tags, 54_ft, 27_ft);

  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(0.3048_m, 0_m, 0.0_m),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

  photon::PhotonPoseEstimator estimator = photon::PhotonPoseEstimator(
      aprilTags, photon::CLOSEST_TO_REFERENCE_POSE,
      photon::PhotonCamera("USB_Camera"), robotToCam);
};
