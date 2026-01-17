// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Helper functions/types for working with Pose2d data. */
public final class PoseHelpers {
  public record PoseDelta(Rotation2d rotationDelta, Distance xDelta, Distance yDelta) {
  }

  /**
   * Computes the difference between a target pose (e.g., how a trajectory is
   * configured to start) and an actual pose (where the robot is currently
   * located/facing on the field).
   *
   * @param targetPose the pose (field position/heading) we want to achieve
   * @param actualPose the pose currently being measured/computed
   * @return the differences in (x,y) position and heading from the target pose
   */
  public static PoseDelta computePoseDelta(
      Pose2d targetPose, Pose2d actualPose) {
    final Rotation2d rotationDelta = targetPose.getRotation().minus(actualPose.getRotation());
    final Distance xDelta = targetPose.getMeasureX().minus(actualPose.getMeasureX());
    final Distance yDelta = targetPose.getMeasureY().minus(actualPose.getMeasureY());
    return new PoseDelta(rotationDelta, xDelta, yDelta);
  }

  /**
   * Computes the angle from a reference pose (e.g., the robot's field position)
   * to a target pose (e.g., something we want to be facing).
   * 
   * @param reference reference pose/position
   * @param target    reference pose/position
   * @return the angle from "reference" to "target"
   */
  public static Angle computeAngleToTarget(Pose2d reference, Pose2d target) {
    double angle = Math.toDegrees(Math.atan2(
        reference.getY() - target.getY(), reference.getX() - target.getX()));

    // Normalize to [0, 360)
    angle = (angle % 360 + 360) % 360;

    return Degrees.of(angle);
  }

  /**
   * Computes the distance from a reference pose (e.g., the robot's field
   * position) to a target pose (e.g., something we want to shoot at/drive to).
   * 
   * @param reference reference pose/position
   * @param target    reference pose/position
   * @return the distance from "reference" to "target"
   */
  public static Distance computeDistanceToTarget(Pose2d reference, Pose2d target) {
    double distanceMeters = Math.hypot(reference.getX() - target.getX(), reference.getY() - target.getY());
    return Meters.of(distanceMeters);
  }
}
