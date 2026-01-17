// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

/** Helper functions/types for working with Pose2d data. */
public final class PoseHelpers {
  public record PoseDelta(Rotation2d rotationDelta, Distance xDelta, Distance yDelta) {}

  /**
   * Computes the difference between a target pose (e.g., how a trajectory is
   * configured to start) and an actual pose (where the robot is currently
   * located/facing on the field).
   *
   * @param targetPose the pose (field position/heading) we want to achieve
   * @param actualPose the pose currently being measured/computed
   * @return the differences in (x,y) position and heading from the target pose
   */
  public static PoseDelta computePoseDelta(Pose2d targetPose, Pose2d actualPose) {
    final Rotation2d rotationDelta = targetPose.getRotation().minus(actualPose.getRotation());
    final Distance xDelta = targetPose.getMeasureX().minus(actualPose.getMeasureX());
    final Distance yDelta = targetPose.getMeasureY().minus(actualPose.getMeasureY());
    return new PoseDelta(rotationDelta, xDelta, yDelta);
  }
}
