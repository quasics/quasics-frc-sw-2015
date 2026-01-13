// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.games;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Defines various constants for the 2025 game, "Reefscape". */
final public class ReefscapeConstants {
  /** Length (x) of the field. */
  static final public Distance FIELD_LENGTH = Meters.of(17.55);

  /** Width (y) of the field. */
  static final public Distance FIELD_WIDTH = Meters.of(8.05);

  /** Field midline (length-wise), where the barge is located. */
  static final public Distance MIDLINE = FIELD_LENGTH.div(2);

  /** Offset from the field midline where the robots are supposed to start. */
  static final public Distance STARTING_LINE_OFFSET = Meters.of(1.2);

  /** Robot distance from east side of the field when on the Blue alliance's starting line. */
  static final public Distance BLUE_STARTING_LINE = FIELD_LENGTH.div(2).minus(STARTING_LINE_OFFSET);

  /** Robot distance from east side of the field when on the Red alliance's starting line. */
  static final public Distance RED_STARTING_LINE = FIELD_LENGTH.div(2).plus(STARTING_LINE_OFFSET);

  /** Robot distance from south side of the field when in front of the "top-most" ball. */
  static final public Distance TOP_BALL_HEIGHT = Meters.of(5.9);

  /** Robot distance from south side of the field when in front of the "middle" ball. */
  static final public Distance MIDDLE_BALL_HEIGHT = Meters.of(4);

  /** Robot distance from south side of the field when in front of the "bottom-most" ball. */
  static final public Distance BOTTOM_BALL_HEIGHT = Meters.of(2.25);
}
