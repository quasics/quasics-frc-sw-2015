// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Defines an interface for retrieval of an estimated pose.
 * 
 * This could be used in the definition of a purely vision-based estimate,
 * or purely odometry, or some fusion of the two. (Or of other sources, of
 * course.)
 */
public interface IPoseEstimator {
    /** @return the estimated pose of the robot */
    public Pose2d getEstimatedPose();
}
