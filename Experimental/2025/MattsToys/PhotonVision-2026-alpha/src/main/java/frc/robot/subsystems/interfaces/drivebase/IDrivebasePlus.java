// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces.drivebase;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.subsystems.interfaces.drivebase.IDrivebase.NullDrivebase;
import frc.robot.utils.BulletinBoard;

/**
 * Adds functionality to the basic IDrivebase interface, such as PID control,
 * pose estimation, and
 * trajectory-following.
 */
public interface IDrivebasePlus extends IDrivebase {
  /**
   * Sets the speeds for the robot.
   *
   * Used for trajectory-following (e.g., with Choreo).
   *
   * @param speeds desired left/right/rotational speeds
   *
   * @see #driveWithPid(DifferentialDriveWheelSpeeds)
   */
  default void driveWithPid(ChassisSpeeds speeds) {
    driveWithPid(getKinematics().toWheelSpeeds(speeds));
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Pose estimation stuff
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** Key used to post odometry-based pose information to BulletinBoard. */
  final String ODOMETRY_KEY = SUBSYSTEM_NAME + ".Pose";

  /** Key used to post odometry-based pose information to BulletinBoard. */
  final String ESTIMATED_POSE_KEY = SUBSYSTEM_NAME + ".PoseEstimate";

  /**
   * Returns the latest posted odemetry-based pose (from the bulletin board).
   *
   * Note: this is a static function, which means that client code doesn't
   * interact directly with
   * the actual subsystem (and thus doesn't need to include it in their
   * requirements).
   *
   * @return last posted odemetry pose, or null
   */
  static Pose2d getPublishedLastPoseFromOdometry() {
    var stored = BulletinBoard.common.getValue(ODOMETRY_KEY, Pose2d.class);
    return (Pose2d) stored.orElse(null);
  }

  /**
   * Returns the latest posted pose estimate (from the bulletin board) based on
   * unified
   * odometry/vision data (from the bulletin board).
   *
   * Note: this is a static function, which means that client code doesn't
   * interact directly with
   * the actual subsystem (and thus doesn't need to include it in their
   * requirements).
   *
   * @return last posted unified pose estimate, or null
   */
  static Pose2d getPublishedLastUnifiedPoseEstimate() {
    var stored = BulletinBoard.common.getValue(ESTIMATED_POSE_KEY, Pose2d.class);
    return (Pose2d) stored.orElse(null);
  }

  /**
   * Gets the robot's pose (based on odometry alone).
   *
   * @return position/heading of the robot, based on odometry
   */
  Pose2d getPose();

  /**
   * Gets the robot's pose (based on pose estimation, fusing odometry and vision).
   *
   * @return estimated pose of the robot
   */
  Pose2d getEstimatedPose();

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Functionality required for AutoBuilder (in PathPlanner library) or
  // AutoFactory (in Choreo library)
  //
  // See: https://choreo.autos/choreolib/getting-started/
  // See: https://www.chiefdelphi.com/t/choreo-2025-beta/472224/23
  // See: https://github.com/mjansen4857/pathplanner/tree/main/examples/java
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Follows (executes) a sample of a Choreo trajectory for a differential drive
   * robot.
   *
   * @param sample component of the trajectory to be followed
   *
   * @see #driveWithPid(ChassisSpeeds)
   */
  void followTrajectory(DifferentialSample sample);

  /**
   * Resets the current pose to the specified value.
   *
   * This should ONLY be called when the robot's position on the field is known
   * (e.g., at the beginning of a match). The code for trajectory-following may
   * invoke this, because it assumes that pre-defined trajectories are started at
   * a well-defined/fixed place.
   *
   *
   * @param pose new pose to use as a basis for odometry/pose estimation
   */
  void resetPose(Pose2d pose);

  /**
   * Sets the wheel speeds (e.g., during trajectory following).
   *
   * Note: as suggested by the name, will use PID control for speed adjustments
   *
   * @param wheelSpeeds desired wheel speeds (based on trajectory planning)
   */
  public void driveWithPid(DifferentialDriveWheelSpeeds wheelSpeeds);

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Trivial implementation
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** Trivial implementation of the IDrivebase interface. */
  public static class NullDrivebase extends IDrivebase.NullDrivebase implements IDrivebasePlus {
    @Override
    public Pose2d getPose() {
      return new Pose2d();
    }

    @Override
    public Pose2d getEstimatedPose() {
      return getPose();
    }

    @Override
    public void followTrajectory(DifferentialSample sample) {
      // No-op
    }

    @Override
    public void resetPose(Pose2d pose) {
      // No-op
    }

    @Override
    public void driveWithPid(DifferentialDriveWheelSpeeds wheelSpeeds) {
      // No-op
    }
  }
}
