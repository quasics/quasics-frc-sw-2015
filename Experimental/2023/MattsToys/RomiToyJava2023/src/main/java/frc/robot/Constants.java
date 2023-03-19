// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. "public static
 * final"). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double TRACK_WIDTH_METERS_ROMI = 0.165;

  /** Constants for speed limits/scaling under various driving modes. */
  public static final class SpeedLimits {
    /** Scaling factor to be applied in "turtle" mode. */
    public static final double MAX_SPEED_TURTLE = 0.50;
    /** Scaling factor to be applied in "normal" mode. */
    public static final double MAX_SPEED_NORMAL = 0.65;
    /** Scaling factor to be applied in "turbo" mode. */
    public static final double MAX_SPEED_TURBO = 0.80;
    /** Scaling factor to be applied in "overdrive" mode. */
    public static final double MAX_SPEED_OVERDRIVE = 1.0;

    /** Max speed overall */
    public static final double ABSOLUTE_LIMIT = MAX_SPEED_TURBO;

    /** Maximum acceleration, in (absolute) units per second. */
    public static final double MAX_SLEW_RATE = 2.0;
  }

  /**
   * Constants associated with "dead zones" on joysticks.
   * 
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  public static final class Deadbands {
    /**
     * Dead band value to be used for deciding when the driver joysticks are at
     * neutral.
     */
    public static final double DRIVING = 0.055;
  }
}
