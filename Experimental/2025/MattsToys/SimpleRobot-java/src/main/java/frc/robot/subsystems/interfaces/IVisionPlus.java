// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Collections;
import java.util.List;

/**
 * Basic interface for vision processing support.
 */
public interface IVisionPlus extends IVision {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  static final String SUBSYSTEM_NAME = "BetterVision";

  /**
   * Key used to post multi-camera List of EstimatedRobotPose data to
   * BulletinBoard.
   */
  static final String POSES_KEY = SUBSYSTEM_NAME + ".EstimatedPoses";

  /** Key used to post last estimated Pose timestamp to BulletinBoard. */
  static final String POSE_TIMESTAMP_KEY = SUBSYSTEM_NAME + ".Timestamp";

  /** Value used to determine "was Pose estimate recently updated?" */
  static final double TIMESTAMP_RECENCY_THRESHOLD_SECS = 0.1;

  List<TargetData> getVisibleTargets();

  /** Trivial implementation of IVisionPlus (e.g., if we don't have a camera). */
  public class NullBetterVision extends IVision.NullVision implements IVisionPlus {
    /** Constructor. */
    public NullBetterVision() {
      System.out.println("INFO: allocating NullBetterVision");
    }

    @Override
    public List<Pose2d> getEstimatedPoses() {
      return Collections.emptyList();
    }

    @Override
    public List<TargetData> getVisibleTargets() {
      return Collections.emptyList();
    }
  }
}
