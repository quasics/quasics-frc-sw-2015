// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Extends IVision to include (implicit promise to) support publishing
 * vision-based pose estimate(s) to BulletinBoard.
 * 
 * @see frc.robot.util.BulletinBoard
 */
public interface IVisionPlus extends IVision {

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Data published to BulletinBoard
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Key used to post a single vision-based EstimatedPoseData record (if
   * supported/available) to BulletinBoard.
   */
  static String ESTIMATED_POSE_KEY = SUBSYSTEM_NAME + ".Pose";

  /**
   * Key used to post a set of vision-based EstimatedPoseData records (if
   * supported/available) to BulletinBoard.
   */
  static String ESTIMATED_POSE_SET_KEY = SUBSYSTEM_NAME + ".PoseSet";

  /**
   * Holds data about an estimated pose for the robot, including a timestamp.
   *
   * @param pose      estimated pose
   * @param timestamp the estimated time the data used to derive the robot pose
   *                  was taken
   */
  record EstimatedPoseData(Pose2d pose, double timestamp) {
  }
}
