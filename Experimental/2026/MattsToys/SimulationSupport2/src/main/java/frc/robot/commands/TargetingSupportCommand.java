// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

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
 * take a shot (e.g., obstacles on the field such as the net in )
 * </ul>
 */
public class TargetingSupportCommand extends Command {
  final ILighting m_lighting;
  final Pose2d m_targetPos;
  final Distance m_maxRange;
  final Distance m_minRange;
  final Angle m_angleRange;

  /** Creates a new TargetingSupportCommand. */
  public TargetingSupportCommand(ILighting lighting, Pose2d targetPos, Distance maxRange, Distance minRange,
      Angle angleRange) {
    m_lighting = lighting;
    m_targetPos = targetPos;
    m_maxRange = maxRange;
    m_minRange = minRange;
    m_angleRange = Degrees.of(Math.abs(angleRange.in(Degrees)));
    addRequirements(m_lighting.asSubsystem());
  }

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
