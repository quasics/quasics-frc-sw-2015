package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Properties;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.io.BufferedWriter;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * TODO(mjh): Consider using reflection to make this easier.
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

    RobotSettings(String robotName, double trackWidthMeters, boolean leftMotorsInverted, boolean rightMotorsInverted) {
        this.robotName = robotName;
        this.trackWidthMeters = trackWidthMeters;
        this.leftMotorsInverted = leftMotorsInverted;
        this.rightMotorsInverted = rightMotorsInverted;
    }

    // Convenience method: will write to a file in the robot's "deploy" directory.
    public boolean writeToFile(String fileName) {
        File f = new File(Filesystem.getDeployDirectory(), fileName);
        try {
            return writeToFile(Files.newBufferedWriter(f.toPath(), Charset.forName("US-ASCII")));
        } catch (java.io.IOException ioe) {
            System.err.format("Error creating writer to save settings to file '%s': %s%n", f.toString(), ioe);
            return false;
        }
    }

    // Convenience method: will load from a file in the robot's "deploy" directory.
    public static RobotSettings loadFromFile(String fileName) {
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
}
