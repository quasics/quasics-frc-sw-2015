package frc.robot.utils.logging;

import frc.robot.utils.NullWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.Writer;

/**
 * TextLogger that will record data to file.
 */
public class FileLogger extends TextLogger {
  /**
   * Constructor.
   *
   * @param f the file to be written to
   * @throws java.io.IOException
   */
  public FileLogger(File f) throws java.io.IOException {
    super(new FileWriter(f));
  }

  /**
   * No-throw constructor.
   *
   * Note: If the underlying file can't be opened for writing, all logging will
   * be handled as no-ops.
   *
   * @param f the file to be written to
   * @param noThrow flag parameter, used to force "no-throw" behavior
   */
  public FileLogger(File f, Object noThrow) {
    super(getWriterNoThrow(f));
  }

  /**
   * Indicates if the object was set up for no-op logging, due a failure during "no-throw"
   * construction.
   *
   * @return true if logging will not actually be written to files
   */
  public boolean isNoOp() {
    return (m_output instanceof NullWriter);
  }

  /**
   * Returns a Writer targeting the specified file, or a NullWriter on failure to target the file
   * for output.
   * @param f file to be written to (on success)
   * @return FileWriter (on successful open) or NullWriter
   */
  static private Writer getWriterNoThrow(File f) {
    try {
      return new FileWriter(f);
    } catch (Exception e) {
      return new NullWriter();
    }
  }
}