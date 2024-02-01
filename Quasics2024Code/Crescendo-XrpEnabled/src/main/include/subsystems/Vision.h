// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
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
      aprilTags,
      // TODO: Josh, I think that CLOSEST_TO_REFERENCE_POSE is only going to
      // work well for you if you're updating the reference pose on a regular
      // basis (e.g., if you're feeding the data from the drive base's estimates
      // back into the vision system on a regular basis).  I would strongly
      // suggest using one of the other options: please talk to me about
      // choices.  (And try testing some of them out under simulation, once you
      // have that in place.)
      photon::CLOSEST_TO_REFERENCE_POSE, photon::PhotonCamera("USB_Camera"),
      robotToCam);
};
