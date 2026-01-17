// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.IVisionPlus;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.util.BulletinBoard;
import frc.robot.util.PoseHelpers;

/**
 * A sample of how we might control the lights to signal when the robot is "in
 * range" to make a shot (given a min/max distance), and facing the target (with
 * some margin of error).
 * 
 * Some notes:
 * <ul>
 * <li>This code assumes that there are no other factors limiting our ability to
 * take a shot (e.g., obstacles on the field such as the net in "Rebuilt").
 * 
 * <li>This code only provides a simple "good/distance bad/angle bad/both
 * bad/don't know" signal, rather than something more nuanced (e.g., "too close
 * and rotated too far clockwise").
 * 
 * <li>We are only using the vision system's estimate for position, which means
 * that if there isn't at least one AprilTag in sight, we can't figure anything
 * out. Using a "fused" estimate (based on both vision and odometry) would be
 * significantly better for a real implementation.
 * 
 * </ul>
 */
public class TargetingSupportCommand extends Command {
  final ILighting m_lighting;
  final Pose2d m_targetPos;
  final Distance m_maxRange;
  final Distance m_minRange;
  final Angle m_angleRange;

  /**
   * Creates a new TargetingSupportCommand.
   * 
   * Note: note that for the "targetPos", we're only using its position, and not
   * the "rotation"/heading. This could be a point for improving the solution in a
   * real-world scenario, where the rotation could indicate the centerline along
   * which it's approachable (e.g., facing blue or red alliance), and then take
   * another Angle measurement to indicate from how broad a range it can be
   * approached. (So that, for example, if you're coming up from behind it, the
   * lights will give different signals than if you're coming in where it's open.)
   * 
   * @param lighting   lighting subsystem
   * @param targetPos  center of the target we're tring to align with, in
   *                   field-relative terms
   * @param maxRange   maximum distance from which we can (likely) make a shot
   * @param minRange   minimum distance from which we can (likely) make a shot
   * @param angleRange margin of error (+/-) that we're willing to accept for
   *                   being "headed for" the target's center
   */
  public TargetingSupportCommand(ILighting lighting, Pose2d targetPos, Distance maxRange, Distance minRange,
      Angle angleRange) {
    m_lighting = lighting;
    m_targetPos = targetPos;
    m_maxRange = maxRange;
    m_minRange = minRange;
    m_angleRange = Degrees.of(Math.abs(angleRange.in(Degrees)));
    addRequirements(m_lighting.asSubsystem());
  }

  /**
   * Updates the lights on the robot, based on the robot's pose vs. the target's
   * location.
   */
  private void update() {
    var positionOpt = BulletinBoard.common.getValue(IVisionPlus.ESTIMATED_POSE_KEY,
        IVisionPlus.EstimatedPoseData.class);
    if (positionOpt.isEmpty()) {
      // Don't know where we are: no status.
      m_lighting.SetStripColor(StockColor.Black);
      return;
    }

    final var robotPose = ((IVisionPlus.EstimatedPoseData) positionOpt.get()).pose();

    Distance distance = PoseHelpers.computeDistanceToTarget(robotPose, m_targetPos);

    // Need to convert "-180 to +180" to "0 to 360"
    final double robotAngleDegrees = robotPose.getRotation().getDegrees() + 180;
    Angle angle = PoseHelpers.computeAngleToTarget(robotPose, m_targetPos);
    final double angleDeltaDegrees = angle.in(Degrees) - robotAngleDegrees;
    final double angleDeltaDegreesAbs = Math.abs(angleDeltaDegrees);

    // Note: heading is treating flat-left as 0 (in -180 to +180), while
    // angleToTarget is treating that as 180 (in 0 to 360)

    System.out.println("Heading: " + robotAngleDegrees + ", to target: " + angle.in(Degrees)
        + ", delta: " + angleDeltaDegreesAbs);

    final boolean angleOK = angleDeltaDegreesAbs <= m_angleRange.in(Degrees);
    final boolean distanceOK = distance.lte(m_maxRange) && distance.gte(m_minRange);
    if (angleOK && distanceOK) {
      m_lighting.SetStripColor(StockColor.Green);
    } else if (angleOK) {
      m_lighting.SetStripColor(StockColor.Purple);
    } else if (distanceOK) {
      m_lighting.SetStripColor(StockColor.Orange);
    } else {
      m_lighting.SetStripColor(StockColor.Red);
    }
  }

  @Override
  public void execute() {
    update();
  }

  @Override
  public void end(boolean interrupted) {
    m_lighting.SetStripColor(StockColor.Black);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
