// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Command to follow a specified (robot-relative) trajectory.
 */
public class FollowTrajectoryCommand extends BaseTrajectoryCommand {
  /** Robot-relative trajectory to be followed. */
  final Trajectory m_baseTrajectory;

  /**
   * Constructor.
   *
   * @param drivebase  drivebase being controlled
   * @param trajectory robot-relative trajectory to be followed
   */
  public FollowTrajectoryCommand(
      IDrivebasePlus drivebase, Trajectory trajectory) {
    super(drivebase);
    m_baseTrajectory = trajectory;
  }

  @Override
  protected Trajectory getTrajectory() {
    // Convert the base trajectory into something relative to the robot's
    // initial pose when the command starts running.
    Transform2d transform = new Transform2d(new Pose2d(), m_drivebase.getEstimatedPose());
    return m_baseTrajectory.transformBy(transform);
  }
}
