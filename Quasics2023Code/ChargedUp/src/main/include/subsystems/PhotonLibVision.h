// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Translation3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/PhotonUtils.h>

#include <map>
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

  bool AprilTagTargetIdentified(int IDWantedTarget);

  std::optional<photonlib::PhotonTrackedTarget> GetIdentifiedAprilTarget(
      int IDWantedTarget);

  bool GetIdentifiedAprilTarget(int IDWantedTarget,
                                photonlib::PhotonTrackedTarget& result);

  bool CalculateDistanceAndAnglesToTarget(int idWantedTarget,
                                          units::meter_t& distance,
                                          units::degree_t& pitchTarget,
                                          units::degree_t& yawTarget);

  frc::Translation3d GetTargetPlacement(int targetID);

  std::optional<photonlib::EstimatedRobotPose> UpdateFieldPosition(
      frc::Pose2d estimatedPose);

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photonlib::PhotonCamera camera{"USB_Camera"};

  std::vector<frc::AprilTag> tags = {
      {586, frc::Pose3d(0_in, 0_in, 17_in, frc::Rotation3d())},
      {0, frc::Pose3d(60_in, 265.0_in, 20.5_in,
                      frc::Rotation3d(0_deg, 0_deg, -90_deg))},
      {1, frc::Pose3d(26.5_in, 253.0_in, 20.5_in,
                      frc::Rotation3d(0_deg, 0_deg, -90_deg))},
      {585, frc::Pose3d(126_in, -62_in, 17_in,
                        frc::Rotation3d(0_deg, 0_deg, 180_deg))}};

  // REPLACE -1 WITH APRIL TAG ID. I DIDNT SEE ANOTHER 6 INCH PRINTED
  // Reply (by matthew): done I added a new april tag id 0  on door

  // 4 inch april tag 583 at (0, 0, 19.1_in)

  // std::shared_ptr<frc::AprilTagFieldLayout> aprilTags =
  //   std::make_shared<frc::AprilTagFieldLayout>(tags, 2000_ft, 2000_ft);
  frc::AprilTagFieldLayout aprilTags =
      frc::AprilTagFieldLayout(tags, 54_ft, 27_ft);
  /*
  std::vector<frc::AprilTag> tags = {
    {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3),
                    frc::Rotation3d())},
    {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5),
                    frc::Rotation3d())}};*/
  /*
    frc::AprilTagFieldLayout aprilTags =
        frc::AprilTagFieldLayout(tags, 54_ft, 27_ft);
    // DOWN TO HERE
  */
  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(0.3048_m, 0_m, 0.0_m),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

  photonlib::PhotonPoseEstimator estimator = photonlib::PhotonPoseEstimator(
      aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE,
      photonlib::PhotonCamera("USB_Camera"), robotToCam);
};
