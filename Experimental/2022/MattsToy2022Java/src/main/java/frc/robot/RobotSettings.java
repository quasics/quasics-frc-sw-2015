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
    static final String TRACK_WIDTH_PROPERTY = "trackWidthMeters";
    static final String ROBOT_NAME_PROPERTY = "robotName";
    public final double trackWidthMeters;
    public final String robotName;

    RobotSettings(String robotName, double trackWidthMeters) {
        this.robotName = robotName;
        this.trackWidthMeters = trackWidthMeters;
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
            return new RobotSettings(name, trackWidth);
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
}
