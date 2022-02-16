package frc.robot;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.io.BufferedWriter;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * TODO(mjh): Consider using reflection to make this easier.
 * TODO(mjh): Consider switching to a USB drive, if needed. These are apparently
 * mounted at "/U" on roboRIO.
 */
public class RobotSettings {
  static final String ROBOT_NAME_PROPERTY = "robotName";
  static final String TRACK_WIDTH_PROPERTY = "trackWidthMeters";
  static final String LEFT_MOTORS_INVERTED_PROPERTY = "leftMotorsInverted";
  static final String RIGHT_MOTORS_INVERTED_PROPERTY = "rightMotorsInverted";

  public final String robotName;
  public final double trackWidthMeters;
  public final boolean leftMotorsInverted;
  public final boolean rightMotorsInverted;

  /**
   * Creates a RobotSettings object.
   * 
   * @param robotName           name of the robot (for debugging/logging)
   * @param trackWidthMeters    track width (m) of the robot
   * @param leftMotorsInverted  iff true, motors on the left side are inverted
   * @param rightMotorsInverted iff true, motors on the right side are inverted
   */
  public RobotSettings(
      String robotName,
      double trackWidthMeters,
      boolean leftMotorsInverted,
      boolean rightMotorsInverted) {
    this.robotName = (robotName != null && robotName.length() > 0 ? robotName : "<unknown>");
    this.trackWidthMeters = trackWidthMeters;
    this.leftMotorsInverted = leftMotorsInverted;
    this.rightMotorsInverted = rightMotorsInverted;
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

  public static List<File> getDeployedPropertyFiles() {
    FilenameFilter filter = (File f, String name) -> {
      return name.endsWith(".prop");
    };
    return Arrays.asList(Filesystem.getOperatingDirectory().listFiles(filter));
  }

  // Convenience method: will write to a file in a consistent directory.
  public boolean writeToFile(String fileName) {
    File f = getPropsFile(fileName);
    try {
      return writeToFile(Files.newBufferedWriter(f.toPath(), Charset.forName("US-ASCII")));
    } catch (java.io.IOException ioe) {
      System.err.format("Error creating writer to save settings to file '%s': %s%n", f.toString(), ioe);
      return false;
    }
  }

  // Convenience method: will load from a file in a consistent directory.
  public static RobotSettings loadFromFile(String fileName) {
    File f = getPropsFile(fileName);
    try {
      return load(new java.io.FileInputStream(f));
    } catch (java.io.FileNotFoundException fnf) {
      System.err.format("Can't find file: %s%n", f.toString());
      return null;
    }
  }

  // Convenience method: will load from a file in the "deploy" directory.
  public static RobotSettings loadFromDeployedFile(String fileName) {
    File f = new File(Filesystem.getDeployDirectory(), fileName);
    try {
      return load(new java.io.FileInputStream(f));
    } catch (java.io.FileNotFoundException fnf) {
      System.err.format("Can't find file: %s%n", f.toString());
      return null;
    }
  }

  public static RobotSettings load(java.io.InputStream in) {
    try (in) {
      Properties props = new Properties();
      props.load(in);
      Double trackWidth = getDoubleFromProperty(props, TRACK_WIDTH_PROPERTY);
      if (trackWidth == null) {
        throw new IOException("Error fetching track width");
      }
      String name = props.getProperty(ROBOT_NAME_PROPERTY);
      if (name == null || name.length() == 0) {
        throw new IOException("Error fetching robot name");
      }
      Boolean leftInverted = getBooleanFromProperty(props, LEFT_MOTORS_INVERTED_PROPERTY);
      if (leftInverted == null) {
        throw new IOException("Error fetching left-side inversion");
      }
      Boolean rightInverted = getBooleanFromProperty(props, RIGHT_MOTORS_INVERTED_PROPERTY);
      if (rightInverted == null) {
        throw new IOException("Error fetching right-side inversion");
      }

      return new RobotSettings(name, trackWidth, leftInverted, rightInverted);
    } catch (IOException ioe) {
      System.err.format("Error loading settings from file: %s%n", ioe);
      return null;
    }
  }

  public boolean writeToFile(BufferedWriter writer) {
    boolean result = false;
    try (writer) {
      Properties props = new Properties();
      props.setProperty(ROBOT_NAME_PROPERTY, robotName);
      props.setProperty(TRACK_WIDTH_PROPERTY, Double.toString(trackWidthMeters));
      props.setProperty(LEFT_MOTORS_INVERTED_PROPERTY, Boolean.toString(leftMotorsInverted));
      props.setProperty(RIGHT_MOTORS_INVERTED_PROPERTY, Boolean.toString(rightMotorsInverted));
      props.store(writer, null);
      result = true;
    } catch (IOException x) {
      System.err.format("Error writing settings to file: %s%n", x);
    }

    return result;
  }

  private static Double getDoubleFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? Double.valueOf(s) : null;
  }

  private static Boolean getBooleanFromProperty(Properties props, String key) {
    String s = props.getProperty(key);
    return (s != null) ? Boolean.valueOf(s) : null;
  }

  @Override
  public String toString() {
    StringBuilder b = new StringBuilder();
    b.append("{");
    b.append("\n  robotName = " + robotName);
    b.append("\n  trackWidthMeters = " + trackWidthMeters);
    b.append("\n  leftMotorsInverted = " + leftMotorsInverted);
    b.append("\n  rightMotorsInverted = " + rightMotorsInverted);
    b.append("\n}");
    return b.toString();
  }

  @Override
  public int hashCode() {
    return robotName.hashCode(); // *Reasonable* uniqueness is all that's required.
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
    return robotName.equals(other.robotName)
        && (trackWidthMeters == other.trackWidthMeters)
        && (leftMotorsInverted == other.leftMotorsInverted)
        && (rightMotorsInverted == other.rightMotorsInverted);
  }
}
