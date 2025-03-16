// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.ILighting.StockColor;

/**
 * A command that sets the color of the candle based on the robot's position on
 * the field, in order to aid the drive team in aiming for the barge.
 * 
 * Note that this example *only* looks at the robot's distance from the barge
 * (well, the center line of the field), and does not take its shooting angle
 * (relative to the barge) into account. (This would be a good improvement to
 * make, and is left as an exercise for the reader.)
 */
public class DriveTeamShootingSupport extends Command {
  /** ICandle being used to show status. */
  private final ICandle m_candle;

  /**
   * Returns true if x is between a and b, inclusive. (Ordering of a and b does
   * not matter.)
   * 
   * @param x the value to check.
   * @param a one bound
   * @param b the other bound
   * 
   * @return true iff x is between a and b, inclusive.
   */
  private static boolean isBetween(Distance x, Distance a, Distance b) {
    return (x.gte(a) && x.lte(b)) ||
        (x.gte(b) && x.lte(a));
  }

  /**
   * Constructor.
   *
   * @param candle The subsystem used by this command.
   */
  public DriveTeamShootingSupport(ICandle candle) {
    m_candle = candle;
    addRequirements(m_candle.asSubsystem());
  }

  /** Center line is 8.77m from the blue alliance wall. */
  final static Distance CENTER_LINE = Meters.of(8.77);
  /** Starting line for blue alliance is ~7.58m from the blue alliance wall. */
  final static Distance BLUE_STARTING_LINE = Meters.of(7.58);
  /** Starting line for red alliance is ~9.96m from the red alliance wall. */
  final static Distance RED_STARTING_LINE = Meters.of(9.96);

  /** How far away from the starting line we can shoot. */
  final static Distance FAR_RANGE = Meters.of(0.0);
  /** How close in to the starting line we can shoot. */
  final static Distance NEAR_RANGE = Meters.of(0.25);

  /** Outer range for when we're on the blue alliance. */
  final static Distance BLUE_FAR_RANGE = BLUE_STARTING_LINE.minus(FAR_RANGE);
  /** Inner range for when we're on the blue alliance. */
  final static Distance BLUE_NEAR_RANGE = BLUE_STARTING_LINE.minus(NEAR_RANGE);
  /** Outer range for when we're on the red alliance. */
  final static Distance RED_FAR_RANGE = RED_STARTING_LINE.plus(FAR_RANGE);
  /** Inner range for when we're on the red alliance. */
  final static Distance RED_NEAR_RANGE = RED_STARTING_LINE.plus(NEAR_RANGE);

  @Override
  public void execute() {
    Pose2d lastPose = IDrivebase.getPublishedLastPose();
    if (lastPose == null) {
      m_candle.setColor(StockColor.Black);
      return;
    }

    var optAlliance = DriverStation.getAlliance();
    DriverStation.Alliance alliance = optAlliance.orElse(null);
    StockColor color = StockColor.Black;
    final Distance xPos = lastPose.getMeasureX();
    if (alliance == DriverStation.Alliance.Blue) {
      if (isBetween(xPos, BLUE_FAR_RANGE, BLUE_NEAR_RANGE)) {
        color = StockColor.Green;
      }
    } else if (alliance == DriverStation.Alliance.Red) {
      if (isBetween(xPos, RED_FAR_RANGE, RED_NEAR_RANGE)) {
        color = StockColor.Green;
      }
    } else {
      // Unknown/unavailable alliance.
      color = StockColor.Black;
    }

    m_candle.setColor(color);
  }

  @Override
  public void end(boolean interrupted) {
    m_candle.setColor(StockColor.Orange);
  }
}
