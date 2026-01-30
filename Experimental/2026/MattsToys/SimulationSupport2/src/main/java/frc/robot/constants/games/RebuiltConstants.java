// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.games;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Defines various constants for the 2026 game, "Rebuilt".
 * 
 * Note that "length" dimensions are along the X-axis (i.e., the long axis) of
 * the field, while "width" dimensions are along the Y-axis (short axis) of the
 * playing field. Values are taken from the published field dimensions and the
 * game manual.
 */
public final class RebuiltConstants {
  /** Robot heading when facing the Blue alliance. */
  public static final Angle FACING_BLUE = Degrees.of(180);

  /** Robot heading when facing the Red alliance. */
  public static final Angle FACING_RED = Degrees.of(0);

  /** Length (x) of the field. */
  public static final Distance FIELD_LENGTH = Inches.of(651.22);

  /** Width (y) of the field. */
  public static final Distance FIELD_WIDTH = Meters.of(8.05);

  /** Field midline (length-wise), where the barge is located. */
  public static final Distance MIDLINE = FIELD_LENGTH.div(2);

  /**
   * Distance (x) from the alliance walls to the center of the "bump" on that
   * alliance's side of the field.
   */
  public static final Distance DRIVER_WALL_TO_BUMP_CENTER = Inches.of(182.11);

  /** Depth (x and y) of the hub (at its base). */
  public static final Distance HUB_DEPTH = Inches.of(47);

  /**
   * Depth (y) of the plexiglass hexagon of the hub (at its top, the shortest
   * measurement across the intake).
   */
  public static final Distance HUB_PLEXIGLASS_DEPTH_Y = Inches.of(41.7);

  /**
   * Depth (x) of the plexiglass hexagon of the hub (at its top, the longest
   * measurement across the intake).
   */
  public static final Distance HUB_PLEXIGLASS_DEPTH_X = HUB_DEPTH;

  /** Length (x) of the trench. */
  public static final Distance TRENCH_LENGTH = Inches.of(47.0);

  /** Width (y) of the trench. */
  public static final Distance TRENCH_WIDTH = Inches.of(65.65);

  /** Distance (x) from the alliance walls to the closest side of the hub. */
  public static final Distance HUB_DISTANCE_TO_ALLIANCE_WALL = Inches.of(158.6);

  /** Distance (x) from the origin corner to the Blue alliance's starting line. */
  public static final Distance BLUE_STARTING_LINE = HUB_DISTANCE_TO_ALLIANCE_WALL;

  /** Distance (x) from the origin corner to the Red alliance's starting line. */
  public static final Distance RED_STARTING_LINE = FIELD_LENGTH.minus(HUB_DISTANCE_TO_ALLIANCE_WALL);

  /** Length (x) of the depot. */
  public static final Distance DEPOT_LENGTH = Inches.of(27.0);

  /** Height (y) of the depot. */
  public static final Distance DEPOT_HEIGHT = Inches.of(42.0);

  /** Length (x) of the bump. */
  public static final Distance BUMP_LENGTH = Inches.of(44.40);

  /** Width (y) of the bump. */
  public static final Distance BUMP_WIDTH = Inches.of(73.0);

  /**
   * Y-position of the mid-point on the lower bumps on the blue/red sides of the
   * field.
   */
  public static final Distance MID_BUMP1_Y = FIELD_WIDTH.div(2).minus(HUB_DEPTH.plus(BUMP_WIDTH).div(2));

  /**
   * Y-position of the mid-point on the upper bumps on the blue/red sides of the
   * field.
   */
  public static final Distance MID_BUMP2_Y = FIELD_WIDTH.div(2).plus(HUB_DEPTH.plus(BUMP_WIDTH).div(2));
}
