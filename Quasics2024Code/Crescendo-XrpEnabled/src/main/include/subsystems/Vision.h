// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>
#include <units/frequency.h>
#include <units/time.h>

#include <cmath>
#include <map>
#include <optional>

#include "Constants.h"
#include "IDrivebase.h"
#include "utils/SimulationSupport.h"

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

  void SimulationPeriodic();

  frc::Field2d& getSimDebugField();

  std::optional<photon::EstimatedRobotPose> getLastEstimatedPose();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::AprilTagFieldLayout aprilTags{
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};

  // TODO: Replace this with an appropriate value for Margaret.
  frc::Transform3d robotToCam{frc::Translation3d(0.3048_m, 0_m, 0.0_m),
                              frc::Rotation3d(0_rad, 0_rad, 0_rad)};

  photon::PhotonPoseEstimator estimator = photon::PhotonPoseEstimator(
      aprilTags,
      // TODO: Josh, I think that CLOSEST_TO_REFERENCE_POSE is only going to
      // work well for you if you're updating the reference pose on a regular
      // basis (e.g., if you're feeding the data from the drive base's estimates
      // back into the vision system on a regular basis).  I would strongly
      // suggest using one of the other options: please talk to me about
      // choices.  (And try testing some of them out under simulation, once you
      // have that in place.)
      photon::LOWEST_AMBIGUITY, photon::PhotonCamera("USB_Camera"), robotToCam);

  // Pose estimator holds the camera; get back a shared pointer to let us
  // set up the PhotonCameraSim.
  std::shared_ptr<photon::PhotonCamera> m_camera = estimator.GetCamera();

  std::unique_ptr<photon::PhotonCameraSim> cameraSim;

  photon::VisionSystemSim visionSim{"main"};

  void setupSimulationSupport();

  void resetSimPose(frc::Pose2d pose);

  void updateEstimatedGlobalPose();

  std::optional<photon::EstimatedRobotPose> m_lastEstimatedPose = std::nullopt;

  units::second_t m_lastEstTimestamp = 0_s;

  bool m_estimateRecentlyUpdated = false;
};
