// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public interface IVision {
  record TargetData(int id, double yaw, double pitch, double distanceToTarget) {
  }

  boolean canSeeTargets();

  List<TargetData> getTargetData();

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);

  Pose2d getVisionLatestPose();
}
