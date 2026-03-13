// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public interface IVision {
  record TargetData(int id, double yaw, double pitch, double distanceToTarget) {
  }

  boolean canSeeTargets();

  List<TargetData> getTargetData();

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);

  Pose2d getVisionLatestPose();

  /**
   * Null implementation of IVision, for use on robots that don't have a camera.
   */
  public class NullVision extends SubsystemBase implements IVision {
    @Override
    public boolean canSeeTargets() {
      return false;
    }

    @Override
    public List<TargetData> getTargetData() {
      return Collections.emptyList();
    }

    @Override
    public void setReferencePositionSupplier(Supplier<Pose2d> supplier) {
      // No-op.
    }

    @Override
    public Pose2d getVisionLatestPose() {
      return null;
    }
  }
}
