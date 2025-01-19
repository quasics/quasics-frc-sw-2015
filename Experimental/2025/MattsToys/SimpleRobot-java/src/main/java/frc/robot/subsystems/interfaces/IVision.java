// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;

public interface IVision {

    public static final String VISION_POSE_KEY = "Vision.Pose";
    public static final String VISION_TIMESTAMP_KEY = "Vision.Timestamp";

    public void updateReferencePose(Pose2d pose);

    public void updateLastPose(Pose2d pose);
}
