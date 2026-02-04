// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import java.util.List;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Example of how we could write a simple (arguably overly-so) command to allow
 * for on-the-fly path generation from any point on the field to a specified
 * location, which could then be tied to a single button for execution.
 * 
 * Note:
 * <ul>
 * <li>For safety reasons, this command should probably only be used under
 * simulation, as it doesn't care about anything that might be in the way on the
 * field (e.g., field elements, other robots, etc.).
 * </ul>
 */
public class PushbuttonTrajectory extends Command {
  /** Drivebase being controlled. (Must support PID control.) */
  final IDrivebasePlus m_drivebase;

  /** Trajectory configuration settings. */
  final TrajectoryConfig m_trajectoryConfig;

  /** Target pose that the command should take us to. */
  final Pose2d m_targetPose;

  /** Timer used to identify samples from the trajectory. */
  final Timer m_timer = new Timer();

  /**
   * Unicycle controller, used to calculate wheel speeds along the way.
   *
   * Notes:
   * <ul>
   * <li>We should consider updating the controller's allocation to also specify
   * the maximum velocity (in m/s).
   *
   * <li>This is the replacement for the RamseteController (deprecated in 2025).
   * </ul>
   */
  final LTVUnicycleController m_controller = new LTVUnicycleController(0.2);

  /**
   * Computed trajectory for the robot, based on where we are when the command
   * starts running, and the end point specified in the constructor.
   */
  Trajectory m_currentTrajectory;

  /**
   * Constructor.
   * 
   * @param drivebase        drivebase being controlled
   * @param trajectoryConfig trajectory-generation configuration/settings
   * @param targetPose       target pose on the field (i.e., where we want to end
   *                         up, and the direction we should be facing)
   */
  public PushbuttonTrajectory(IDrivebasePlus drivebase, TrajectoryConfig trajectoryConfig, Pose2d targetPose) {
    m_drivebase = drivebase;
    m_trajectoryConfig = trajectoryConfig;
    m_targetPose = targetPose;

    addRequirements(drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    // Build a trajectory from where we *are* to where we *want to be*.
    Pose2d startingPose = m_drivebase.getEstimatedPose();
    m_currentTrajectory = TrajectoryGenerator.generateTrajectory(
        // Starting where we are right now...
        startingPose,
        // No interior waypoints - just a simplest path/straight line
        List.of(),
        // End point
        m_targetPose,
        // Pass config
        m_trajectoryConfig);

    // Restart the timer (used for sampling in execute()).
    m_timer.restart();
  }

  @Override
  public void execute() {
    // Calculate how fast the wheels should be moving at this point along the
    // trajectory
    double elapsed = m_timer.get();
    var referencePosition = m_currentTrajectory.sample(elapsed);
    ChassisSpeeds newSpeeds = m_controller.calculate(
        m_drivebase.getEstimatedPose(), referencePosition);

    // Move the drivebase accordingly.
    m_drivebase.driveTankWithPID(newSpeeds);
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
