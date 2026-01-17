package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;

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
