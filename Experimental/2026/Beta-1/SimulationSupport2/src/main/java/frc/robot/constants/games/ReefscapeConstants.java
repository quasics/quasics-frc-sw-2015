package frc.robot.constants.games;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

final public class ReefscapeConstants {
  final static public Distance FIELD_LENGTH = Meters.of(17.55);
  final static public Distance FIELD_WIDTH = Meters.of(8.05);
  final static public Distance MIDLINE = FIELD_LENGTH.div(2);
  final static public Distance STARTING_LINE_OFFSET = Meters.of(1.2);

  /** Robot distance from east side of the field when on the Blue alliance's starting line. */
  final static public Distance BLUE_STARTING_LINE = FIELD_LENGTH.div(2).minus(STARTING_LINE_OFFSET);

  /** Robot distance from east side of the field when on the Red alliance's starting line. */
  final static public Distance RED_STARTING_LINE = FIELD_LENGTH.div(2).plus(STARTING_LINE_OFFSET);

  /** Robot distance from south side of the field when in front of the "top-most" ball. */
  final static public Distance TOP_BALL_HEIGHT = Meters.of(5.9);

  /** Robot distance from south side of the field when in front of the "middle" ball. */
  final static public Distance MIDDLE_BALL_HEIGHT = Meters.of(4);

  /** Robot distance from south side of the field when in front of the "bottom-most" ball. */
  final static public Distance BOTTOM_BALL_HEIGHT = Meters.of(2.25);
}
