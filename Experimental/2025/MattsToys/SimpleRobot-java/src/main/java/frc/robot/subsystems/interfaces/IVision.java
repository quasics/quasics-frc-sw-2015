// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/**
 * Basic interface for vision processing support.
 */
public interface IVision extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  static final String SUBSYSTEM_NAME = "Vision";

  /** Key used to post a single-camera estimated Pose to BulletinBoard. */
  static final String VISION_SINGLE_POSE_KEY = SUBSYSTEM_NAME + ".Pose";

  /** Key used to post multi-camera estimated Poses to BulletinBoard. */
  static final String VISION_MULTI_POSE_KEY = SUBSYSTEM_NAME + ".Poses";

  /** Key used to post last estimated Pose timestamp to BulletinBoard. */
  static final String VISION_TIMESTAMP_KEY = SUBSYSTEM_NAME + ".Timestamp";

  /** Key used to post "was Pose estimate recently updated?" to BulletinBoard. */
  static final double VISION_TIMESTAMP_RECENCY_THRESHOLD_SECS = 0.01;

  /** Estimate data reported in the multi-camera case. */
  public record EstimateResult(EstimatedRobotPose pose, double timestamp) {
  }

  ////////////////////////////////////////////////////////////////////////
  //
  // Single-camera functions.
  //
  // TODO: Consider removing these, or adding multi-camera functions.
  // TODO: Consider rewriting these to use "EstimateResult".
  //
  ////////////////////////////////////////////////////////////////////////

  /** @return last estimated pose (if any), based on vision-tracking data */
  Optional<EstimatedRobotPose> getLastEstimatedPose();

  /** @return timestamp for last data used to update estimated pose */
  double getLastEstTimestamp();

  /** @return true iff the estimated pose was recently updated */
  boolean getEstimateRecentlyUpdated();

  /** Trivial implementation of IVision (e.g., if we don't have a camera). */
  public class NullVision implements IVision {
    /** Constructor. */
    public NullVision() {
      System.out.println("INFO: allocating NullVision");
    }

    @Override
    public Optional<EstimatedRobotPose> getLastEstimatedPose() {
      return Optional.empty();
    }

    @Override
    public double getLastEstTimestamp() {
      return 0;
    }

    @Override
    public boolean getEstimateRecentlyUpdated() {
      return false;
    }
  }
}
