// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() {
  SetName("Vision");
}

// This method will be called once per scheduler run
void Vision::updateEstimatedGlobalPose() {
  if (camera.GetCameraName() == NULL) {
    return;
  }

  m_lastEstimatedPose = estimator.Update();

  units::second_t latestTimestamp = camera.GetLatestResult().GetTimestamp();
  m_estimateRecentlyUpdated =
      std::abs(latestTimestamp.value() - m_lastEstTimestamp.value()) > 1e-5;

  if (m_estimateRecentlyUpdated) {
    m_lastEstTimestamp = latestTimestamp;
  }
}

std::optional<photon::EstimatedRobotPose> Vision::getLastEstimatedPose() {
  return m_lastEstimatedPose;
}

void Vision::Periodic() {
  updateEstimatedGlobalPose();
}

void Vision::SimulationPeriodic() {
  auto possiblePose = SimulationSupport::getSimulatedPose();
  if (possiblePose.has_value()) {
    visionSim.Update(possiblePose.value());
  }

  /*if (Robot::IsSimulation()) {
    if(m_lastEstimatedPose.has_value()){
      getSimDebugField()
          .GetObject("VisionEstimation")
          .SetPose(est.estimatedPose.toPose2d());
    }
    else {
      if(m_estimateRecentlyUpdated)
        getSimDebugField().GetObject("VisionEstimation").setPose();
    }
  }*/
}

bool Vision::AprilTagTargetIdentified(int IDWantedTarget) {
  photon::PhotonPipelineResult ID = camera.GetLatestResult();
  if (!ID.HasTargets()) {
    // Didn't see anything.
    return false;
  }

  std::span<const photon::PhotonTrackedTarget> targets = ID.GetTargets();
  for (const photon::PhotonTrackedTarget target : targets) {
    if (target.GetFiducialId() == IDWantedTarget) {
      // Found it!
      return true;
    }
  }

  // Couldn't find it.
  return false;
}

std::optional<photon::PhotonTrackedTarget> Vision::GetIdentifiedAprilTarget(
    int IDWantedTarget) {
  photon::PhotonPipelineResult result = camera.GetLatestResult();
  if (!result.HasTargets()) {
    // Didn't see anything.
    return std::nullopt;
  }

  for (const photon::PhotonTrackedTarget& target : result.GetTargets()) {
    if (target.GetFiducialId() == IDWantedTarget) {
      // Found it!
      return target;
    }
  }

  // Couldn't find it.
  return std::nullopt;
}

bool Vision::CalculateDistanceAndAnglesToTarger(int IDWantedTarget,
                                                units::meter_t& distance,
                                                units::degree_t& pitchTarget,
                                                units::degree_t& yawTarget) {
  auto possibleTarget = GetIdentifiedAprilTarget(IDWantedTarget);
  if (!possibleTarget.has_value()) {
    return false;
  }

  auto target = possibleTarget.value();
  const units::degree_t pitchToTarget(target.GetPitch());
  const units::degree_t yawToTarget(target.GetYaw());
  units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      1_in, 1_in, 0_rad, pitchToTarget);

  distance = range;
  pitchTarget = pitchToTarget;
  yawTarget = yawToTarget;
  return true;
}

std::optional<photon::EstimatedRobotPose> Vision::UpdateFieldPosition(
    frc::Pose2d estimatedPose) {
  // TODO: Josh, this isn't going to do what you think, unless you've configured
  // the pose estimator to use
  estimator.SetReferencePose(frc::Pose3d(estimatedPose));
  return estimator.Update();
}

void Vision::setupSimulationSupport() {
  if (Robot::IsReal()) {
    return;
  }

  visionSim.AddAprilTags(aprilTags);

  photon::SimCameraProperties cameraProp;
  cameraProp.SetCalibration(960, 720, 90_deg);
  cameraProp.SetCalibError(0.35, 0.10);
  cameraProp.SetFPS(units::frequency::hertz_t(15));
  cameraProp.SetAvgLatency(50_ms);
  cameraProp.SetLatencyStdDev(15_ms);

  cameraSim.reset(new photon::PhotonCameraSim(&camera, cameraProp));

  visionSim.AddCamera(cameraSim.get(), robotToCam);

  cameraSim->EnableDrawWireframe(true);
}

void Vision::resetSimPose(frc::Pose2d pose) {
  if (Robot::IsSimulation()) {
    visionSim.ResetRobotPose(pose);
  }
}

frc::Field2d& Vision::getSimDebugField() {
  return visionSim.GetDebugField();
}