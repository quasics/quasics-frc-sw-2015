// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. "public static final"). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double INCHES_PER_METER = 39.3701;

  public static final double TRACK_WIDTH_METERS_ROMI = 0.165;

  // TODO(mjh): Confirm this value for Sally....
  public static final double TRACK_WIDTH_INCHES_SALLY = 22.0;

  // TODO(mjh): Confirm this value for Mae....
  /** 2021 robot's width, from in-code constants (which are in meters) */
  public static final double TRACK_WIDTH_INCHES_MAE = 47.134344149315914763;

  // TODO(mjh): Confirm this for Nike
  public static final double TRACK_WIDTH_INCHES_NIKE = TRACK_WIDTH_INCHES_MAE;

  public static final double WHEEL_DIAMETER_INCHES = 6.0;

  /** Gear ratio for 2017 robot, kept as spare drive base. */
  public static final double DRIVE_BASE_GEAR_RATIO_NIKE = 10.71;
  /** 2020/2021 robot gear ratio. */
  public static final double DRIVE_BASE_GEAR_RATIO_MAE = 10.71;
  /** 2022 robot gear rato (swapped in new gearbox vs. 10.71 in KoP) */
  public static final double DRIVE_BASE_GEAR_RATIO_SALLY = 8.45;
  /** 2023 robot gear ratio (KoP gearing is 8.45:1 this year) */
  public static final double DRIVE_BASE_GEAR_RATIO_2023 = 8.45;

  public static final double DRIVE_BASE_GEAR_RATIO = DRIVE_BASE_GEAR_RATIO_SALLY;

  public static final int PIGEON2_CAN_ID = 1;

  public static final class MotorIds {
    public static final class SparkMax {
      public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
      public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2;
      public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
      public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
    }
  }

  /** Constants for speed limits/scaling under various driving modes. */
  public static final class SpeedLimits {
    /** Scaling factor to be applied in "turtle" mode. */
    public static final double MAX_SPEED_TURTLE = 0.50;
    /** Scaling factor to be applied in "normal" mode. */
    public static final double MAX_SPEED_NORMAL = 0.65;
    /** Scaling factor to be applied in "turbo" mode. */
    public static final double MAX_SPEED_TURBO = 0.80;

    /** Max speed overall */
    public static final double ABSOLUTE_LIMIT = MAX_SPEED_TURBO;

    /** Maximum acceleration, in (absolute) units per second. */
    public static final double MAX_SLEW_RATE = 0.5;
  }

  /**
   * Constants associated with "dead zones" on joysticks.
   *
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  public static final class Deadbands {
    /** Dead band value to be used for deciding when the driver joysticks are at neutral. */
    public static final double DRIVING = 0.055;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public enum Alliance {
    Red,
    Blue
  }

  public enum RobotPosition {
    eRed1(Alliance.Red, 1),
    eRed2(Alliance.Red, 2),
    eRed3(Alliance.Red, 3),
    eBlue1(Alliance.Blue, 1),
    eBlue2(Alliance.Blue, 2),
    eBlue3(Alliance.Blue, 3);

    RobotPosition(Alliance alliance, int position) {
      this.m_pos = position;
      this.m_alliance = alliance;
    }

    public int position() {
      return m_pos;
    }

    public Alliance alliance() {
      return m_alliance;
    }

    public String toString() {
      StringBuilder b = new StringBuilder();
      b.append(m_alliance);
      b.append(" ");
      b.append(m_pos);
      return b.toString();
    }

    private static final double CLOSE_COMMUNITY_LINE_METERS = 3.36;
    private static final double FAR_COMMUNITY_LINE_METERS = 4.91;

    public double distanceToCommunityLineInMeters() {
      // TODO: Check that the position #s are right.
      if ((m_alliance == Alliance.Red && m_pos == 3)
          || (m_alliance == Alliance.Blue && m_pos == 1)) {
        return CLOSE_COMMUNITY_LINE_METERS;
      } else {
        // Note that this doesn't take into account additional distance for getting over the
        // charging station.  A *real* implementation would need to do so.
        return FAR_COMMUNITY_LINE_METERS;
      }
    }

    private final int m_pos;
    private final Alliance m_alliance;
  }
}
