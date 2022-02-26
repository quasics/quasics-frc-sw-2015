package frc.robot;

import java.io.File;
import java.io.FilenameFilter;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.io.BufferedWriter;

import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;
import frc.robot.utils.TrajectoryCommandGenerator.SpeedProfile;

import edu.wpi.first.wpilibj.Filesystem;

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
 * 
 * TODO(mjh): Break this apart into something that manages the robot settings,
 * and a utility class to handle loading/storing them.
 * TODO(mjh): Add gear ratio to this.
 */
public class RobotSettings {
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

  public final double driveProfileKs;
  public final double driveProfileKv;
  public final double driveProfileKa;

  public final double driveControlKp;
  public final double driveControlKi;
  public final double driveControlKd;

  public RobotSettings(
      String robotName,
      double trackWidthMeters,
      double gearRatio,
      DriveProfileData profileData,
      PIDConfig pidConfig,
      boolean leftMotorsInverted,
      boolean rightMotorsInverted,
      GyroType installedGyroType,
      int pigeonCanId) {
    this(robotName,
        trackWidthMeters,
        gearRatio,
        profileData.kS,
        profileData.kV,
        profileData.kA,
        pidConfig.kP,
        pidConfig.kI,
        pidConfig.kD,
        leftMotorsInverted,
        rightMotorsInverted,
        installedGyroType,
        pigeonCanId);
  }

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
      double driveProfileKs,
      double driveProfileKv,
      double driveProfileKa,
      double driveControlKp,
      double driveControlKi,
      double driveControlKd,
      boolean leftMotorsInverted,
      boolean rightMotorsInverted,
      GyroType installedGyroType,
      int pigeonCanId) {
    this.robotName = (robotName != null && robotName.length() > 0 ? robotName : "<unknown>");
    this.trackWidthMeters = trackWidthMeters;
    this.gearRatio = gearRatio;

    this.driveProfileKs = driveProfileKs;
    this.driveProfileKv = driveProfileKv;
    this.driveProfileKa = driveProfileKa;

    this.driveControlKp = driveControlKp;
    this.driveControlKi = driveControlKi;
    this.driveControlKd = driveControlKd;

    this.leftMotorsInverted = leftMotorsInverted;
    this.rightMotorsInverted = rightMotorsInverted;
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

    Boolean b = getBooleanFromProperty(props, "leftMotorsInverted");
    if (b == null) {
      throw new IllegalArgumentException("Error fetching left-side inversion");
    }
    this.leftMotorsInverted = b;

    b = getBooleanFromProperty(props, "rightMotorsInverted");
    if (b == null) {
      throw new IllegalArgumentException("Error fetching right-side inversion");
    }
    this.rightMotorsInverted = b;

    GyroType g = getGyroTypeFromProperty(props, "installedGyroType");
    if (g == null) {
      throw new IllegalArgumentException("Error fetching installed gyro type");
    }
    this.installedGyroType = g;

    // The "pigeonCanId" field is only valid if we're working *with* a Pigeon2.
    if (this.installedGyroType == GyroType.Pigeon2) {
      Integer canId = getIntegerFromProperty(props, "pigeonCanId");
      if (canId == null) {
        throw new IllegalArgumentException("Error fetching CAN ID for installed Pigeon2");
      }
      this.pigeonCanId = canId;
    } else {
      this.pigeonCanId = 0;
    }
  }

  /**
   * Converts a simple filename to a File object, in a well-defined directory.
   * 
   * Note: the "deploy" directory doesn't appear to be writeable by the robot
   * programs.
   */
  private static File getPropsFile(String fileName) {
    return new File(Filesystem.getOperatingDirectory(), fileName);
  }

  /**
   * @return a list of the property files (assumed to be RobotSettings storage) in
   *         the "deploy" directory
   */
  public static List<File> getDeployedPropertyFiles() {
    FilenameFilter filter = (File f, String name) -> {
      return name.endsWith(".props");
    };
    return Arrays.asList(Filesystem.getOperatingDirectory().listFiles(filter));
  }

  /**
   * Convenience method: will write to a file in a consistent directory.
   * 
   * @see #getPropsFile(String)
   * @see #writeToFile(BufferedWriter)
   */
  public boolean writeToFile(String fileName) {
    File f = getPropsFile(fileName);
    try {
      return writeToFile(Files.newBufferedWriter(f.toPath(), Charset.forName("US-ASCII")));
    } catch (java.io.IOException ioe) {
      System.err.format("Error creating writer to save settings to file '%s': %s%n", f.toString(), ioe);
      return false;
    }
  }

  /**
   * Writes the data for the object to the specified writer, using the standard
   * "Properties" format.
   * 
   * @param writer the sink to which the data should be stored
   * @return true on success, false on any failure
   * 
   * @see #buildProperties()
   * @see java.util.Properties#store(java.io.Writer, String)
   */
  public boolean writeToFile(BufferedWriter writer) {
    boolean result = false;
    try (writer) {
      Properties props = buildProperties();
      props.store(writer, "Data for robot '" + robotName);
      result = true;
    } catch (Exception x) {
      System.err.format("Error saving settings: %s%n", x);
    }

    return result;
  }

  /**
   * Convenience method: will load from a file in a well-defined directory.
   * 
   * @see #getPropsFile(String)
   * @see #load(java.io.InputStream)
   */
  public static RobotSettings loadFromFile(String fileName) {
    File f = getPropsFile(fileName);
    try {
      return load(new java.io.FileInputStream(f));
    } catch (java.io.FileNotFoundException fnf) {
      System.err.format("Can't find file: %s%n", f.toString());
      return null;
    }
  }

  /**
   * Convenience method: will load from a file in the "deploy" directory.
   * 
   * @see #load(java.io.InputStream)
   */
  public static RobotSettings loadFromDeployedFile(String fileName) {
    File f = new File(Filesystem.getDeployDirectory(), fileName);
    try {
      return load(new java.io.FileInputStream(f));
    } catch (java.io.FileNotFoundException fnf) {
      System.err.format("Can't find file: %s%n", f.toString());
      return null;
    }
  }

  /**
   * Returns a RobotSettings object using properties read from the specified input
   * stream.
   * 
   * @param in the InputStream from which the data is to be loaded
   * @return a RobotSettings object on success, null on failure
   * 
   * @seee {@link #RobotSettings(Properties)}
   */
  public static RobotSettings load(java.io.InputStream in) {
    Properties props = new Properties();
    try (in) {
      props.load(in);
      return new RobotSettings(props);
    } catch (Exception e) {
      System.err.format("---------------------------------------%n"
          + "Error loading settings: %s%n"
          + "Data loaded (leading to failure) was: %s%n"
          + "---------------------------------------%n", e, props);
      return null;
    }
  }

  /**
   * Builds a Properties object containing all of the data represented by this
   * object, with the keys being the names of each data field, and the values
   * being a string version of the coresponding field value.
   * 
   * @return a Properties object holding key/value pairs for all of this object's
   *         data
   * @throws IllegalArgumentException
   * @throws IllegalAccessException
   */
  private Properties buildProperties() throws IllegalArgumentException, IllegalAccessException {
    Properties props = new Properties();
    Class<?> myClass = getClass();
    Field[] fields = myClass.getFields();
    for (var field : fields) {
      props.setProperty(field.getName(), field.get(this).toString());
    }

    return props;
  }

  private static double getCheckedDouble(Properties props, String propName, String errorText) {
    Double d = getDoubleFromProperty(props, "propName");
    if (d == null) {
      throw new IllegalArgumentException(errorText);
    }
    return d;
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
  public String toString() {
    try {
      return buildProperties().toString();
    } catch (Exception e) {
      return "<Error: Failed to extract data for robot settings>";
    }
  }

  @Override
  public int hashCode() {
    if (robotName != null) {
      return robotName.hashCode(); // *Reasonable* uniqueness is all that's required.
    }
    return 0;
  }

  @Override
  public boolean equals(Object o) {
    if (o == null) {
      return false;
    }
    if (!(o instanceof RobotSettings)) {
      return false;
    }

    RobotSettings other = (RobotSettings) o;
    try {
      return this.buildProperties().equals(other.buildProperties());
    } catch (IllegalArgumentException | IllegalAccessException e) {
      return false;
    }
  }
}
