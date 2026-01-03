// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Command to follow a specified trajectory.
 */
public class FollowTrajectoryCommand extends Command {
  final IDrivebasePlus m_drivebase;
  final Trajectory m_baseTrajectory;
  final Timer m_timer = new Timer();
  Trajectory m_currentTrajectory;

  /**
   * Unicycle controller, used to calculate wheel speeds along the way.
   *
   * Notes:
   * <ul>
   * <li>We should consider updating the controller's allocation to also specify the maximum
   * velocity (in m/s).
   *
   * <li>This is the replacement for the RamseteController (deprecated in 2025).
   * </ul>
   */
  final LTVUnicycleController m_controller = new LTVUnicycleController(0.2);

  /**
   * Constructor.
   *
   * @param drivebase   drivebase being controlled
   * @param trajectory  trajectory to be followed (assumed to be robot-relative)
   */
  public FollowTrajectoryCommand(IDrivebasePlus drivebase, Trajectory trajectory) {
    m_drivebase = drivebase;
    m_baseTrajectory = trajectory;
    addRequirements(m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    m_timer.restart();

    // Convert the base trajectory into something relative to the robot's initial pose when the
    // command starts running.
    Transform2d transform = new Transform2d(new Pose2d(), m_drivebase.getEstimatedPose());
    m_currentTrajectory = m_baseTrajectory.transformBy(transform);
  }

  @Override
  public void execute() {
    // Calculate how fast we should be moving at this point along the trajectory
    double elapsed = m_timer.get();
    var referencePosition = m_currentTrajectory.sample(elapsed);

    // OK, let's do that
    ChassisSpeeds newSpeeds =
        m_controller.calculate(m_drivebase.getEstimatedPose(), referencePosition);
    m_drivebase.setSpeeds(newSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_currentTrajectory.getTotalTimeSeconds());
  }
}
