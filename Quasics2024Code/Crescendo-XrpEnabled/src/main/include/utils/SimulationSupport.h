// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Pose2d.h>

#include <optional>

class SimulationSupport {
 public:
  SimulationSupport();

  static std::optional<frc::Pose2d> getSimulatedPose() {
    return simulatedPose;
  }

  /*static void setSimulatedPose(frc::Pose2d pose) {
    if() {
      simulatedPose =
    }
  }*/

 private:
  static std::optional<frc::Pose2d> simulatedPose;
};
