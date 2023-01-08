package frc.robot.utils;

import java.util.Properties;

import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;

/**
 * Used to hold various characteristics that vary between the robots that we
 * have on hand (e.g., drive base width, motor inversions), allowing the code to
 * adapt to the hardware on which it is deployed.
 * 
 * Note: We are currently using either the "operating" or "deploy" directories
 * when reading/writing files holding settings data. If we want, another option
 * would be to switch to a USB drive (which might make "default values" easier
 * to deploy to a given robot). These are apparently mounted at "/U" by the
 * roboRIO firmware image.
 */
public class RobotSettings extends PropertyBasedObject {
  public enum DriveMotorInversion {
    None(false, false),
    Left(true, false),
    Right(false, true),
    Both(true, true);

    final public boolean leftInverted;
    final public boolean rightInverted;

    DriveMotorInversion(boolean leftInverted, boolean rightInverted) {
      this.leftInverted = leftInverted;
      this.rightInverted = rightInverted;
    }
  }

  /** Different types of FRC gyros understood as part of robot settings data. */
  public enum GyroType {
    None,
    ADXRS450,
    Pigeon2,
    Romi
  }

  /** Name of the robot (for debugging). */
  public final String robotName;

  /** Track width of the robot (e.g., for path following support). */
  public final double trackWidthMeters;

  public final double gearRatio;

  /**
   * Iff true, the motors on the left side of the drive base are installed in an
   * inverted configuration.
   */
  public final boolean leftMotorsInverted;

  /**
   * Iff true, the motors on the right side of the drive base are installed in an
   * inverted configuration.
   */
  public final boolean rightMotorsInverted;

  /** The type of gyro installed on this robot (if any). */
  public final GyroType installedGyroType;

  /** The CAN ID for the installed gyro, if it is a Pigeon2. */
  public final int pigeonCanId;

  /** kS constant for drive base profile (obtained from SysId tool). */
  public final double driveProfileKs;
  /** kS constant for drive base profile (obtained from SysId tool). */
  public final double driveProfileKv;
  /** kA constant for drive base profile (obtained from SysId tool). */
  public final double driveProfileKa;

  /** kP constant for drive base PID control (obtained from SysId tool). */
  public final double driveControlKp;
  /** kI constant for drive base PID control (obtained from SysId tool). */
  public final double driveControlKi;
  /** kD constant for drive base PID control (obtained from SysId tool). */
  public final double driveControlKd;

  /**
   * Creates a RobotSettings object.
   * 
   * Note: this constructor is expected to be used only for building settings in
   * the RobotContainer class either for immediate use in writing them to an
   * active file, or as a default set to use when we can't load stuff *from* a
   * file. The preferred way to get RobotSettings is to load them from a file
   * (either a "deployed" one, or from a previous save point).
   * 
   * @param robotName           name of the robot (for debugging/logging)
   * @param trackWidthMeters    track width (m) of the robot
   * @param gearRatio           the gear ratio on the drive base
   * @param profileData         drive profiling constants (kS/kV/kA)
   *                            characterizing the robot's drive base (e.g., with
   *                            SysId)
   * @param driveControl        PID values derived from characterizing the robot's
   *                            drive base (e.g., with SysId)
   * @param leftMotorsInverted  iff true, drive motors on the left side are
   *                            inverted
   * @param rightMotorsInverted iff true, drive motors on the right side are
   *                            inverted
   * @param installedGyroType   the type of gyro on the robot
   * @param pigeonCanId         the CAN ID for the gyro, if it's a Pigeon2 (or any
   *                            value, if it's not)
   * 
   * @see #load(java.io.InputStream)
   */
  public RobotSettings(
      String robotName,
      double trackWidthMeters,
      double gearRatio,
      DriveProfileData profileData,
      PIDConfig pidConfig,
      DriveMotorInversion motorInversion,
      GyroType installedGyroType,
      int pigeonCanId) {
    this.robotName = (robotName != null && robotName.length() > 0 ? robotName : "<unknown>");
    this.trackWidthMeters = trackWidthMeters;
    this.gearRatio = gearRatio;

    this.driveProfileKs = profileData.kS;
    this.driveProfileKv = profileData.kV;
    this.driveProfileKa = profileData.kA;

    this.driveControlKp = pidConfig.kP;
    this.driveControlKi = pidConfig.kI;
    this.driveControlKd = pidConfig.kD;

    this.leftMotorsInverted = motorInversion.leftInverted;
    this.rightMotorsInverted = motorInversion.rightInverted;
    this.installedGyroType = installedGyroType;
    this.pigeonCanId = pigeonCanId;
  }

  /**
   * Creates a RobotSettings object.
   * 
   * @param props a Properties object from which the robot settings should be
   *              retrieved
   * 
   * @throws IllegalArgumentException
   */
  public RobotSettings(Properties props) throws IllegalArgumentException {
    String s = props.getProperty("robotName");
    if (s == null || s.length() == 0) {
      throw new IllegalArgumentException("Error fetching robot name");
    }
    this.robotName = s;

    this.trackWidthMeters = getCheckedDouble(props, "trackWidthMeters", "Error fetching track width");
    this.gearRatio = getCheckedDouble(props, "gearRatio", "Error fetching gear ratio");

    this.driveProfileKs = getCheckedDouble(props, "driveProfileKs", "Error fetching kS value for drive profile");
    this.driveProfileKv = getCheckedDouble(props, "driveProfileKv", "Error fetching kV value for drive profile");
    this.driveProfileKa = getCheckedDouble(props, "driveProfileKa", "Error fetching kA value for drive profile");

    this.driveControlKp = getCheckedDouble(props, "driveControlKp", "Error fetching kP value for drive control");
    this.driveControlKi = getCheckedDouble(props, "driveControlKi", "Error fetching kI value for drive control");
    this.driveControlKd = getCheckedDouble(props, "driveControlKd", "Error fetching kD value for drive control");

    this.leftMotorsInverted = getCheckedBoolean(props, "leftMotorsInverted", "Error fetching left-side inversion");
    this.rightMotorsInverted = getCheckedBoolean(props, "rightMotorsInverted", "Error fetching right-side inversion");

    GyroType g = getGyroTypeFromProperty(props, "installedGyroType");
    if (g == null) {
      throw new IllegalArgumentException("Error fetching installed gyro type");
    }
    this.installedGyroType = g;

    // The "pigeonCanId" field is only valid if we're working *with* a Pigeon2.
    if (this.installedGyroType == GyroType.Pigeon2) {
      this.pigeonCanId = getCheckedInteger(props, "pigeonCanId", "Error fetching CAN ID for installed Pigeon2");
    } else {
      this.pigeonCanId = 0;
    }
  }

  private static double getCheckedDouble(Properties props, String propName, String errorText) {
    Double d = getDoubleFromProperty(props, "propName");
    if (d == null) {
      throw new IllegalArgumentException(errorText);
    }
    return d;
  }

  private static boolean getCheckedBoolean(Properties props, String propName, String errorText) {
    Boolean b = getBooleanFromProperty(props, "propName");
    if (b == null) {
      throw new IllegalArgumentException(errorText);
    }
    return b;
  }

  private static int getCheckedInteger(Properties props, String propName, String errorText) {
    Integer i = getIntegerFromProperty(props, "propName");
    if (i == null) {
      throw new IllegalArgumentException(errorText);
    }
    return i;
  }

  private static Double getDoubleFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? Double.valueOf(s) : null;
  }

  private static Integer getIntegerFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? Integer.valueOf(s) : null;
  }

  private static Boolean getBooleanFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? Boolean.valueOf(s) : null;
  }

  private static GyroType getGyroTypeFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? GyroType.valueOf(s) : null;
  }

  ////////////////////////////////////////
  // Methods from "Object"

  @Override
  public int hashCode() {
    if (robotName != null) {
      return robotName.hashCode(); // *Reasonable* uniqueness is all that's required.
    }
    return 0;
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof RobotSettings)) {
      return false;
    }
    return super.equals(o);
  }
}
