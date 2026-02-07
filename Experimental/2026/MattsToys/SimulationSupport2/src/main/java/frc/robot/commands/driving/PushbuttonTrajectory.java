// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.constants.games.RebuiltConstants;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import java.util.List;
import org.opencv.core.Point;
import org.opencv.core.Rect2d;

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
 * force termination if we were in an unsafe/unsuitable region of the field). I
 * have included a simple example of this type of check in this command, and it
 * is indirectly supported by the BaseTrajectoryCommand class.
 * </ul>
 */
public class PushbuttonTrajectory extends BaseTrajectoryCommand {
  /** Trajectory configuration settings. */
  final TrajectoryConfig m_trajectoryConfig;

  /** Target pose that the command should take us to. */
  final Pose2d m_targetPose;

  /**
   * Constructor.
   *
   * @param drivebase        drivebase being controlled
   * @param trajectoryConfig trajectory-generation configuration/settings
   * @param targetPose       target pose on the field (i.e., where we want to
   *     end
   *                         up, and the direction we should be facing)
   */
  public PushbuttonTrajectory(IDrivebasePlus drivebase,
      TrajectoryConfig trajectoryConfig, Pose2d targetPose) {
    super(drivebase);
    m_trajectoryConfig = trajectoryConfig;
    m_targetPose = targetPose;
  }

  /** (Roughly) defines the Blue alliance's end of the field. */
  final static Rect2d BLUE_ZONE =
      new Rect2d(0, 0, RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
          RebuiltConstants.FIELD_LENGTH.in(Meters));

  /** (Roughly) defines the Red alliance's end of the field. */
  final static Rect2d RED_ZONE = new Rect2d(
      // X, Y
      RebuiltConstants.RED_STARTING_LINE.in(Meters), 0,
      // Width, height
      RebuiltConstants.FIELD_WIDTH.minus(RebuiltConstants.RED_STARTING_LINE)
          .in(Meters),
      RebuiltConstants.FIELD_LENGTH.in(Meters));

  /**
   * @return true iff the specified point is contained within a target
   *     rectangle.
   */
  private static final boolean isPoseInRect(Pose2d pose, Rect2d rect) {
    return rect.contains(new Point(pose.getX(), pose.getY()));
  }

  /**
   * Simple test to see if the robot's path should only lie in a "known safe"
   * region. (Note that this is only an *example*; code for the playing field
   * would likely need to be more complex.)
   *
   * @return true if the start and end points are both in the blue zone or the
   *     red
   *         zone
   */
  protected boolean robotIsInSafeArea() {
    Pose2d curPose = m_drivebase.getEstimatedPose();
    if (isPoseInRect(m_targetPose, BLUE_ZONE)
        && isPoseInRect(curPose, BLUE_ZONE)) {
      return true;
    } else if (isPoseInRect(m_targetPose, RED_ZONE)
        && isPoseInRect(curPose, RED_ZONE)) {
      return true;
    }
    return false;
  }

  @Override
  protected Trajectory getTrajectory() {
    final Pose2d startingPose = m_drivebase.getEstimatedPose();
    if (robotIsInSafeArea()) {
      System.out.println("Running pushbutton trajectory from " + startingPose
          + " to " + m_targetPose);

      // Build a trajectory from where we *are* to where we *want to be*.
      return TrajectoryGenerator.generateTrajectory(
          // Starting where we are right now...
          startingPose,
          // No interior waypoints - just a simplest path/straight line
          List.of(),
          // End point
          m_targetPose,
          // Pass config
          m_trajectoryConfig);
    } else {
      System.err.println("Can't run pushbutton trajectory from " + startingPose
          + " to " + m_targetPose);
      return null;
    }
  }
}
