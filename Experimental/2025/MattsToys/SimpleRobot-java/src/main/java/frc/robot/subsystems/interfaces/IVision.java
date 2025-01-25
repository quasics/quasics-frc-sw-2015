// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Basic interface for vision processing support.
 */
public interface IVision {
    /** Name for the subsystem (and base for BulletinBoard keys). */
    static final String SUBSYSTEM_NAME = "Vision";

    /** Key used to post estimated Pose to BulletinBoard. */
    static final String VISION_POSE_KEY = SUBSYSTEM_NAME + ".Pose";

    /** Key used to post last estimated Pose timestamp to BulletinBoard. */
    static final String VISION_TIMESTAMP_KEY = SUBSYSTEM_NAME + ".Timestamp";

    /** Key used to post "was Pose estimate recently updated?" to BulletinBoard. */
    static final double VISION_TIMESTAMP_RECENCY_THRESHOLD_SECS = 0.01;

    /**
     * Updates the reference pose (e.g., from the drivebase, for use in building
     * visual estimates).
     */
    void updateReferencePose(Pose2d pose);

    /**
     * Updates the last pose (e.g., from the drivebase, for use in building visual
     * estimates).
     */
    void updateLastPose(Pose2d pose);

    /** @return last estimated pose (if any), based on vision-tracking data */
    Optional<EstimatedRobotPose> getLastEstimatedPose();

    /** @return timestamp for last data used to update estimated pose */
    double getLastEstTimestamp();

    /** @return true iff the estimated pose was recently updated */
    boolean getEstimateRecentlyUpdated();
}
