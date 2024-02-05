// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

#include <units/angle.h>
#include <units/length.h>

#include <iostream>

#include "subsystems/IDrivebase.h"

constexpr double kMinGapBetweenUpdates = 0.00001;
constexpr std::string_view kVisionEstimateObjectName{"VisionEstimate"};

Vision::Vision() {
  SetName("Vision");
  // m_visionSim.AddCamera(&m_cameraSim, kRobotToCam);
  // m_visionSim.AddAprilTags(m_fieldLayout);
  // m_cameraSim.EnableDrawWireframe(ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO);

  // std::cout << "-------------------------------------------------\n"
  //           << "Vision simulator is configured.  URLs are:\n"
  //           << "* http://localhost:1181 (raw stream)\n"
  //           << "* http://localhost:1182 (processed stream)\n"
  //           << "Wireframe rendering is "
  //           << (ENABLE_WIREFRAME_RENDERING_ON_RAW_VIDEO ? "" : "not")
  //           << "enabled\n"
  //           << "-------------------------------------------------";
}

// This method will be called once per scheduler run
void Vision::Periodic() {
  SubsystemBase::Periodic();

  // updateEstimatedGlobalPose();
}

void Vision::SimulationPeriodic() {
  SubsystemBase::SimulationPeriodic();

  // // Update the vision simulation with the current position computed by the
  // // drivebase tracking.
  // m_visionSim.Update(IDrivebase::GetDrivebase().getPose());

  // // Update the simulation data for the field shown in the simulator.
  // frc::Field2d& field = m_visionSim.GetDebugField();
  // frc::FieldObject2d* fieldEstObject =
  //     field.GetObject(kVisionEstimateObjectName);
  // if (m_lastEstimatedPose.has_value()) {
  //   fieldEstObject->SetPose(
  //       m_lastEstimatedPose.value().estimatedPose.ToPose2d());
  // } else {
  //   if (m_estimatedPoseRecentlyUpdated) {
  //     fieldEstObject->SetPoses({});
  //   };
  // }
}

void Vision::updateEstimatedGlobalPose() {
  // m_lastEstimatedPose = m_photonEstimator.Update();
  // auto latestTimestamp = m_camera->GetLatestResult().GetTimestamp();
  // m_estimatedPoseRecentlyUpdated =
  //     std::abs((latestTimestamp - m_lastEstTimestamp).value());
  // if (m_estimatedPoseRecentlyUpdated) {
  //   m_lastEstTimestamp = latestTimestamp;
  // }
}