// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.games;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Supported (pre-defined) starting positions for the robot.
 *
 * Note that game-specific starting positions are pre-fixed with the game name
 * (e.g., "Reefscape_").
 */
public enum StartingPosition {
  /** Default robot position (0, 0, 0). */
  Default,

  //
  // Sample starting positions for Rebuilt (2026).
  //

  /** Facing Blue and aligned with the middle of the lower trench in Rebuilt. */
  Rebuilt_BlueStart1,

  /** Facing Red and aligned with the middle of the lower trench in Rebuilt. */
  Rebuilt_RedStart1,

  //
  // Sample starting positions for Reefscape (2025).
  //

  /** Facing Blue and aligned with starting game element 1 in Reefscape. */
  Reefscape_Blue1,
  /** Facing Blue and aligned with starting game element 2 in Reefscape. */
  Reefscape_Blue2,
  /** Facing Blue and aligned with starting game element 3 in Reefscape. */
  Reefscape_Blue3,
  /** Facing Red and aligned with starting game element 1 in Reefscape. */
  Reefscape_Red1,
  /** Facing Red and aligned with starting game element 2 in Reefscape. */
  Reefscape_Red2,
  /** Facing Red and aligned with starting game element 3 in Reefscape. */
  Reefscape_Red3,
  Reefscape_Extra1,
  Reefscape_Extra2,
  ;

  /** Default starting pose for the robot. */
  public static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(0, 0, new Rotation2d());

  /** Returns the robot pose associated with this starting point. */
  public Pose2d getPose() {

    return switch (this) {
      case Default -> DEFAULT_STARTING_POSE;

      case Rebuilt_BlueStart1 ->
        new Pose2d(
            RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
            RebuiltConstants.TRENCH_WIDTH.div(2).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_BLUE));

      case Rebuilt_RedStart1 ->
        new Pose2d(
            RebuiltConstants.RED_STARTING_LINE.in(Meters),
            RebuiltConstants.TRENCH_WIDTH.div(2).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_RED));

      case Reefscape_Blue1 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape_Blue2 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape_Blue3 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape_Red1 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));
      case Reefscape_Red2 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));
      case Reefscape_Red3 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));

      case Reefscape_Extra1 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) - .5,
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) - .5,
            new Rotation2d(
                ReefscapeConstants.FACING_BLUE.minus(Degrees.of(5))));
      case Reefscape_Extra2 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) + .5,
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) + .5,
            new Rotation2d(
                ReefscapeConstants.FACING_BLUE.plus(Degrees.of(5))));
    };
  }

  /**
   * Returns the user-facing name of this starting position (e.g., for use in
   * a chooser).
   */
  public String getName() {
    return this.toString().replaceFirst(".*__", "");
  }
}
