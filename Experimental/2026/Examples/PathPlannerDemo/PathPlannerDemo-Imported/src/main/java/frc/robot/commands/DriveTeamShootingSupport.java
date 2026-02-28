// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.utils.SupportFunctions.isBetween;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.subsystems.interfaces.drivebase.IDrivebasePlus;

/**
 * A command that sets the color of the candle based on the robot's position on
 * the field, in order to aid the drive team in aiming for the barge.
 *
 * Note that this example *only* looks at the robot's distance from the barge
 * (well, the center line of the field), and does not take its shooting angle
 * (relative to the barge) into account. (This would be a good improvement to
 * make, and is left as an exercise for the reader.)
 *
 * Please also note that the values (and colors) I've chosen for this example
 * are somewhat arbitrary, and do not reflect discussions with the Quasics drive
 * team (or other sources of reality-based data).
 */
public class DriveTeamShootingSupport extends Command {
  /** ICandle being used to show status. */
  private final ICandle m_candle;

  /**
   * Constructor.
   *
   * @param candle The subsystem used by this command.
   */
  public DriveTeamShootingSupport(ICandle candle) {
    m_candle = candle;
    addRequirements(m_candle.asSubsystem());
  }

  /** Color to show if we can't get data (alliance/pose). */
  final static StockColor ERROR = StockColor.Black;
  /** Color to show if we aren't in position for a shot. */
  final static StockColor NO_SHOT = StockColor.Black;
  /** Color to show if the range is good, but the angle is not. */
  final static StockColor ANGLE_BAD = StockColor.Orange;
  /** Color to show if the angle is good, but the range is not. */
  final static StockColor RANGE_BAD = StockColor.Silver;
  /** Color to show if we're in position for a shot. */
  final static StockColor IN_POSITON = StockColor.Green;
  /** Color to revert to when the command stops running. */
  final static StockColor DISABLED = StockColor.Purple;

  @Override
  public void execute() {
    // Make sure we have data to work with.
    Pose2d lastPose = IDrivebasePlus.getPublishedLastPoseFromOdometry();
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(null);
    if (lastPose == null || alliance == null) {
      m_candle.setColor(ERROR);
      return;
    }

    final Distance xPos = lastPose.getMeasureX();
    final Angle angle = lastPose.getRotation().getMeasure();

    // Figure out how close we are in terms of distance and angle.
    final boolean distanceOk = switch (alliance) {
      case Blue -> {
        yield isBetween(xPos, BLUE_FAR_SHOOTING_RANGE, BLUE_NEAR_SHOOTING_RANGE);
      }
      case Red -> {
        yield isBetween(xPos, RED_FAR_SHOOTING_RANGE, RED_NEAR_SHOOTING_RANGE);
      }
      default -> {
        yield false;
      }
    };

    final boolean angleOk = switch (alliance) {
      case Blue -> {
        yield isBetween(angle, BLUE_FAR_SHOOTING_ANGLE, BLUE_NEAR_SHOOTING_ANGLE);
      }
      case Red -> {
        yield isBetween(angle, RED_FAR_SHOOTING_ANGLE, RED_NEAR_SHOOTING_ANGLE);
      }
      default -> {
        yield false;
      }
    };

    // Pick the color, based on pose evaluation.
    StockColor color = ERROR;
    if (distanceOk && angleOk) {
      color = IN_POSITON;
    } else if (distanceOk) {
      color = ANGLE_BAD;
    } else if (angleOk) {
      color = RANGE_BAD;
    } else {
      color = NO_SHOT;
    }

    m_candle.setColor(color);
  }

  @Override
  public void end(boolean interrupted) {
    m_candle.setColor(DISABLED);
  }
}
