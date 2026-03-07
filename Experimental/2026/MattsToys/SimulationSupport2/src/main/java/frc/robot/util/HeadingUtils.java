package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility functions related to heading calculations.
 */
public class HeadingUtils {
  /**
   * Calculates the change in rotation needed to align the robot.
   * 
   * @param currentPose   The robot's current pose from odometry.
   * @param targetHeading The field-relative heading to align with.
   * @return Rotation2d representing the angular error (should be constrained to
   *         -180 to +180).
   */
  public static Rotation2d getRotationError(Pose2d currentPose, Rotation2d targetHeading) {
    // Get the current field-relative heading from the pose.
    Rotation2d currentHeading = currentPose.getRotation();

    // Subtract current from target to find the "error".
    // (Target - Current = How much further we need to go)
    return targetHeading.minus(currentHeading);
  }
}
