// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.opencv.core.Point;
import org.opencv.core.Rect2d;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.games.RebuiltConstants;
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
 * 
 * <li>The preceding is because WPILib's TrajectoryGenerator does not "pathfind"
 * around constraints automatically. For example, if you tried to specify a
 * RectangularRegionConstraint and set the maximum velocity within that area to
 * 0, and the straightline path identified by TrajectoryGenerator ran through
 * it, the generator will likely throw an error or produce an impossible path.
 * As a result, handling a more complex case (e.g., avoiding field elements)
 * would also require writing code to identify such potential problems and then
 * defining waypoints that moved around them, that could be specified in the
 * interiorWaypoints list passed into the TrajectoryGenerator.
 * 
 * <li>On the other hand, this could work as something thqt could simply be
 * triggered by the drive team when the robot was in some general region of the
 * field (and could include a safeguard test for this, either in a precondition,
 * or in "isFinished()", or even as part of a ParallelCommandGroup that could
 * force termination if we were in an unsafe/unsuitable region of the field,)
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

  /** (Roughly) defines the Blue alliance's end of the field. */
  final static Rect2d BLUE_ZONE = new Rect2d(0, 0,
      RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
      RebuiltConstants.FIELD_LENGTH.in(Meters));

  /** (Roughly) defines the Red alliance's end of the field. */
  final static Rect2d RED_ZONE = new Rect2d(
      // X, Y
      RebuiltConstants.RED_STARTING_LINE.in(Meters), 0,
      // Width, height
      RebuiltConstants.FIELD_WIDTH.minus(RebuiltConstants.RED_STARTING_LINE).in(Meters),
      RebuiltConstants.FIELD_LENGTH.in(Meters));

  /**
   * @return true iff the specified point is contained within a target rectangle.
   */
  private static final boolean isPoseInRect(Pose2d pose, Rect2d rect) {
    return rect.contains(new Point(pose.getX(), pose.getY()));
  }

  /**
   * Simple test to see if the robot's path should only lie in a "known safe"
   * region. (Note that this is only an *example*; code for the playing field
   * would likely need to be more complex.)
   * 
   * @return true if the start and end points are both in the blue zone or the red
   *         zone
   */
  protected boolean robotIsInSafeArea() {
    Pose2d curPose = m_drivebase.getEstimatedPose();
    if (isPoseInRect(m_targetPose, BLUE_ZONE) && isPoseInRect(curPose, BLUE_ZONE)) {
      return true;
    } else if (isPoseInRect(m_targetPose, RED_ZONE) && isPoseInRect(curPose, RED_ZONE)) {
      return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    final Pose2d startingPose = m_drivebase.getEstimatedPose();
    if (robotIsInSafeArea()) {
      // Build a trajectory from where we *are* to where we *want to be*.
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
      System.out.println("Running pushbutton trajectory from " + startingPose
          + " to " + m_targetPose);
    } else {
      m_timer.stop();
      m_currentTrajectory = null;
      System.err.println("Can't run pushbutton trajectory from " + startingPose
          + " to " + m_targetPose);
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
    return !m_timer.isRunning() || m_timer.hasElapsed(m_currentTrajectory.getTotalTimeSeconds());
  }
}
