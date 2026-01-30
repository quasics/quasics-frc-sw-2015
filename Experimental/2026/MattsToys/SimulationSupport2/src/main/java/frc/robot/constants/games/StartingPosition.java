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
 * (e.g., "Reefscape__").
 */
public enum StartingPosition {
  /** Default robot position (0, 0, 0). */
  Default,

  //
  // Sample starting positions for Rebuilt (2026).
  //

  /** Facing Blue and aligned with the middle of the lower trench in Rebuilt. */
  Rebuilt__BlueMidTrench1,
  Rebuilt__BlueMidTrench2,
  Rebuilt__BlueMidBump1,
  Rebuilt__BlueMidBump2,

  /** Facing Red and aligned with the middle of the lower trench in Rebuilt. */
  Rebuilt__RedMidTrench1,
  Rebuilt__RedMidTrench2,
  Rebuilt__RedMidBump1,
  Rebuilt__RedMidBump2,

  //
  // Sample starting positions for Reefscape (2025).
  //

  /** Facing Blue and aligned with starting game element 1 in Reefscape. */
  Reefscape__Blue1,
  /** Facing Blue and aligned with starting game element 2 in Reefscape. */
  Reefscape__Blue2,
  /** Facing Blue and aligned with starting game element 3 in Reefscape. */
  Reefscape__Blue3,
  /** Facing Red and aligned with starting game element 1 in Reefscape. */
  Reefscape__Red1,
  /** Facing Red and aligned with starting game element 2 in Reefscape. */
  Reefscape__Red2,
  /** Facing Red and aligned with starting game element 3 in Reefscape. */
  Reefscape__Red3, Reefscape__Extra1, Reefscape__Extra2,;

  /** Default starting pose for the robot. */
  public static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(0, 0, new Rotation2d());

  /** Returns the robot pose associated with this starting point. */
  public Pose2d getPose() {

    return switch (this) {
      case Default -> DEFAULT_STARTING_POSE;

      case Rebuilt__BlueMidTrench1 ->
        new Pose2d(
            RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
            RebuiltConstants.TRENCH_WIDTH.div(2).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_BLUE));
      case Rebuilt__BlueMidTrench2 ->
        new Pose2d(
            RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
            RebuiltConstants.FIELD_WIDTH.minus(RebuiltConstants.TRENCH_WIDTH.div(2)).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_BLUE));
      case Rebuilt__BlueMidBump1 ->
        new Pose2d(
            RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
            RebuiltConstants.MID_BUMP1_Y.in(Meters),
            new Rotation2d(RebuiltConstants.FACING_BLUE));
      case Rebuilt__BlueMidBump2 ->
        new Pose2d(
            RebuiltConstants.BLUE_STARTING_LINE.in(Meters),
            RebuiltConstants.MID_BUMP2_Y.in(Meters),
            new Rotation2d(RebuiltConstants.FACING_BLUE));

      case Rebuilt__RedMidTrench1 ->
        new Pose2d(
            RebuiltConstants.RED_STARTING_LINE.in(Meters),
            RebuiltConstants.TRENCH_WIDTH.div(2).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_RED));
      case Rebuilt__RedMidTrench2 ->
        new Pose2d(
            RebuiltConstants.RED_STARTING_LINE.in(Meters),
            RebuiltConstants.FIELD_WIDTH.minus(RebuiltConstants.TRENCH_WIDTH.div(2)).in(Meters),
            new Rotation2d(RebuiltConstants.FACING_RED));
      case Rebuilt__RedMidBump1 ->
        new Pose2d(
            RebuiltConstants.RED_STARTING_LINE.in(Meters),
            RebuiltConstants.MID_BUMP1_Y.in(Meters),
            new Rotation2d(RebuiltConstants.FACING_RED));
      case Rebuilt__RedMidBump2 ->
        new Pose2d(
            RebuiltConstants.RED_STARTING_LINE.in(Meters),
            RebuiltConstants.MID_BUMP2_Y.in(Meters),
            new Rotation2d(RebuiltConstants.FACING_RED));

      case Reefscape__Blue1 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape__Blue2 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape__Blue3 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
            ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_BLUE));
      case Reefscape__Red1 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));
      case Reefscape__Red2 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));
      case Reefscape__Red3 ->
        new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
            new Rotation2d(ReefscapeConstants.FACING_RED));

      case Reefscape__Extra1 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) - .5,
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) - .5,
            new Rotation2d(
                ReefscapeConstants.FACING_BLUE.minus(Degrees.of(5))));
      case Reefscape__Extra2 ->
        new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) + .5,
            ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) + .5,
            new Rotation2d(
                ReefscapeConstants.FACING_BLUE.plus(Degrees.of(5))));
    };
  }

  /**
   * Returns the user-facing name of this starting position (e.g., for use in
   * a chooser).
   * 
   * @see #getName(boolean)
   */
  public String getName() {
    return getName(true);
  }

  /**
   * Returns the user-facing name of this starting position (e.g., for use in
   * a chooser).
   * 
   * @param stripGameFromName if true, remove any game-specific prefix (e.g.,
   *                          "Reefscape__") from the name
   */
  public String getName(boolean stripGameFromName) {
    if (stripGameFromName) {
      return toString().replaceFirst(".*__", "");
    } else {
      return toString();
    }
  }

  /**
   * Returns the user-facing name of this starting position, optionally removing a
   * game-specific prefix.
   * 
   * @param game if non-null, the game for which prefixes should be removed from
   *             names
   */
  public String getNameWithoutGamePrefix(Game game) {
    final String rawName = toString();
    if (game == null) {
      return rawName;
    }

    final String prefixString = game.toString() + "__";
    if (rawName.startsWith(prefixString)) {
      return rawName.substring(prefixString.length());
    }

    return rawName;
  }
}
