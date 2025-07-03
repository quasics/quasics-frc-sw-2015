// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.util.Collections;
import java.util.List;
import org.photonvision.EstimatedRobotPose;

/**
 * Basic interface for vision processing support.
 */
public interface IBetterVision extends IVision {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  static final String SUBSYSTEM_NAME = "Vision";

  /**
   * Key used to post multi-camera List of EstimatedRobotPose data to
   * BulletinBoard.
   */
  static final String POSES_KEY = SUBSYSTEM_NAME + ".EstimatedPoses";

  /** Key used to post last estimated Pose timestamp to BulletinBoard. */
  static final String POSE_TIMESTAMP_KEY = SUBSYSTEM_NAME + ".Timestamp";

  /** Value used to determine "was Pose estimate recently updated?" */
  static final double TIMESTAMP_RECENCY_THRESHOLD_SECS = 0.1;

  /**
   * Returns the most recent pose estimates, based on camera data.
   *
   * @return the most recent pose estimates
   */
  List<EstimatedRobotPose> getEstimatedPoses();

  /** Trivial implementation of IVision (e.g., if we don't have a camera). */
  public class NullVision implements IBetterVision {
    /** Constructor. */
    public NullVision() {
      System.out.println("INFO: allocating NullVision");
    }

    @Override
    public List<EstimatedRobotPose> getEstimatedPoses() {
      return Collections.emptyList();
    }

    @Override
    public boolean hasTargetsInView() {
      return false;
    }

    @Override
    public List<TargetData> getTargets() {
      return Collections.emptyList();
    }
  }
}
