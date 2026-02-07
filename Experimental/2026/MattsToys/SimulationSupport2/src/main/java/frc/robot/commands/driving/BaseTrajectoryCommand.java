// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Base class for commands that provide relatively simple support for trajectory
 * following.
 *
 * Derived classes will need to implement the "getTrajectory()" method, which
 * will be called during command initialization (i.e., when the command is
 * triggered) to get the trajectory that should be followed (or null, if there
 * is a problem/no available path).
 */
public abstract class BaseTrajectoryCommand extends Command {
  /** Drivebase being controlled. (Must support PID control.) */
  final IDrivebasePlus m_drivebase;

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
   * Trajectory for the robot to follow while the command is running.
   *
   * @see #getTrajectory()
   */
  Trajectory m_currentTrajectory;

  /**
   * Constructor.
   *
   * @param drivebase drivebase being controlled
   */
  public BaseTrajectoryCommand(IDrivebasePlus drivebase) {
    m_drivebase = drivebase;

    addRequirements(drivebase.asSubsystem());
  }

  /**
   * Function used by derived classes to provide the trajectory to be followed.
   * (Invoked each time that the command is started.)
   *
   * @return the trajectory to be followed by the robot, or null if no
   *     trajectory
   *         can be computed
   *
   * @see #initialize()
   */
  protected abstract Trajectory getTrajectory();

  @Override
  public void initialize() {
    m_currentTrajectory = getTrajectory();
    if (m_currentTrajectory != null) {
      // Restart the timer.
      m_timer.restart();
    } else {
      // Can't generate a trajectory: nothing to do.
      m_timer.stop();
    }
  }

  @Override
  public void execute() {
    if (!m_timer.isRunning() || m_currentTrajectory == null) {
      return;
    }

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
    return !m_timer.isRunning()
        || m_timer.hasElapsed(m_currentTrajectory.getTotalTimeSeconds());
  }
}
