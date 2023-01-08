package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;

public class PropsIO {
  /**
   * Converts a simple filename to a File object, in a well-defined directory.
   * 
   * Note: the "deploy" directory doesn't appear to be writeable by the robot
   * programs.
   */
  public static File getPropsFile(String fileName) {
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
   * @throws IOException
   * @throws IllegalAccessException
   * @throws IllegalArgumentException
   * 
   * @see #getPropsFile(String)
   * @see #writeToFile(BufferedWriter)
   */
  public static void writeToFile(PropertyBasedObject o, String fileName, String comment)
      throws IOException, IllegalArgumentException, IllegalAccessException {
    File f = getPropsFile(fileName);
    writeToFile(o.buildProperties(), Files.newBufferedWriter(f.toPath(), Charset.forName("US-ASCII")), comment);
  }

  /**
   * Convenience method: will write to a file in a consistent directory.
   * 
   * @throws IOException
   * 
   * @see #getPropsFile(String)
   * @see #writeToFile(BufferedWriter)
   */
  public static void writeToFile(Properties props, String fileName, String comment) throws IOException {
    File f = getPropsFile(fileName);
    writeToFile(props, Files.newBufferedWriter(f.toPath(), Charset.forName("US-ASCII")), comment);
  }

  /**
   * Writes the data for the object to the specified writer, using the standard
   * "Properties" format.
   * 
   * @param writer the sink to which the data should be stored
   * @throws IOException
   * 
   * @see #buildProperties()
   * @see java.util.Properties#store(java.io.Writer, String)
   */
  public static void writeToFile(Properties props, BufferedWriter writer, String comment) throws IOException {
    try (writer) {
      props.store(writer, comment);
    }
  }

  /**
   * Convenience method: will load from a file in a well-defined directory.
   * 
   * @throws IOException
   * @throws FileNotFoundException
   * 
   * @see #getPropsFile(String)
   * @see #load(java.io.InputStream)
   */
  public static Properties loadFromFile(String fileName) throws FileNotFoundException, IOException {
    File f = getPropsFile(fileName);
    return load(new java.io.FileInputStream(f));
  }

  /**
   * Convenience method: will load from a file in the "deploy" directory.
   * 
   * @throws IOException
   * @throws FileNotFoundException
   * 
   * @see #load(java.io.InputStream)
   */
  public static Properties loadFromDeployedFile(String fileName) throws FileNotFoundException, IOException {
    File f = new File(Filesystem.getDeployDirectory(), fileName);
    return load(new java.io.FileInputStream(f));
  }

  /**
   * Returns a RobotSettings object using properties read from the specified input
   * stream.
   * 
   * @param in the InputStream from which the data is to be loaded
   * @return a RobotSettings object on success, null on failure
   * @throws IOException
   * 
   * @seee {@link #RobotSettings(Properties)}
   */
  public static Properties load(java.io.InputStream in) throws IOException {
    Properties props = new Properties();
    try (in) {
      props.load(in);
    }
    return props;
  }
}
