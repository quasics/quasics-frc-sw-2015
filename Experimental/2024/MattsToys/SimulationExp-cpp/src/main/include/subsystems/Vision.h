// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>
#include <units/time.h>

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  ////////////////////////////////////////////////////////////
  // Overriding methods from our base class.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  ////////////////////////////////////////////////////////////
  // Helper functions.
 private:
  /**
   * Determines any updated estimates on the position based on camera data.
   *
   * Note: This calculation is somewhat expensive, but should be done once per
   * cycle, hence we cache the results in data members.
   *
   * @see #Periodic()
   * @see #m_lastEstimatedPose
   * @see #m_lastEstTimestamp
   * @see #m_estimatedPoseRecentlyUpdated
   */
  void updateEstimatedGlobalPose();

  ////////////////////////////////////////////////////////////
  // Data members.
 private:
  // Note: camera name, robot-to-camera transform, and simulated camera
  // properties are all hard-coded.  We might want to make them more like
  // the camera we're really using on "production" code, but for this
  // sample, that's less of an issue.
  const std::string m_cameraName{"vision-sim"};
  const frc::Transform3d kRobotToCam{
      frc::Translation3d(/*x=*/0.5_m, /*y=*/0.0_m, /*z=*/0.5_m),
      frc::Rotation3d(/*roll=*/0_deg, /*pitch=*/-30_deg, /*yaw=*/0_deg)};
  const photon::SimCameraProperties m_simCameraProperties =
      photon::SimCameraProperties::PI4_LIFECAM_640_480();
  const bool ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO = true;

  const frc::AprilTagFieldLayout m_fieldLayout =
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  photon::PhotonPoseEstimator m_photonEstimator{
      m_fieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      // Note: camera must be passed into pose estimator as an r-value reference
      // due to the constructor signature for the class.  (Unclear why it
      // doesn't take a shared_ptr to the camera, allowing sharing with the
      // subsystem.)
      std::move(photon::PhotonCamera{m_cameraName}), kRobotToCam};

  // Pose estimator holds the camera; get back a shared pointer to let us
  // set up the PhotonCameraSim.
  std::shared_ptr<photon::PhotonCamera> m_camera =
      m_photonEstimator.GetCamera();

  // Handles camera simulation.
  photon::PhotonCameraSim m_cameraSim{m_camera.get(), m_simCameraProperties};

  // Handles overall vision system simulation.
  photon::VisionSystemSim m_visionSim{m_cameraName};

  std::optional<photon::EstimatedRobotPose> m_lastEstimatedPose{};
  units::time::second_t m_lastEstTimestamp{};
  bool m_estimatedPoseRecentlyUpdated = false;
};
