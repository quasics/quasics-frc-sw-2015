// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.File;
import java.io.FileWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.time.Instant;

/** Add your docs here. */
public abstract class EventLogger {
  //////////////////////////////////////////////////////////////////////
  //
  // Static stuff
  //
  //////////////////////////////////////////////////////////////////////

  /** A default logger instance, for convenient access. */
  private static EventLogger m_defaultLogger = new TextLogger(new NullWriter());

  /**
   * Returns the default logger.
   * @return the default logger
   */
  public static synchronized EventLogger instance() {
    return m_defaultLogger;
  }

  /**
   * Sets a new default logger (which will be a no-op logger, if the provided value is null).
   *
   * @param logger new default logger; if null, a no-op logger will be instantiated
   * @return the new default logger
   */
  public static synchronized EventLogger setDefaultLogger(EventLogger logger) {
    if (logger == null) {
      logger = new TextLogger(new NullWriter());
    }
    m_defaultLogger = logger;
    m_defaultLogger.logText("New default logger registered");

    return m_defaultLogger;
  }

  //////////////////////////////////////////////////////////////////////
  //
  // Instance "stuff"
  //
  //////////////////////////////////////////////////////////////////////

  /** Writer used by this logger. */
  final protected Writer m_output;

  /**
   * Constructor.
   *
   * @param output destination for logged data
   */
  protected EventLogger(Writer output) {
    m_output = output;
  }

  /**
   * Convenice logging function.
   *
   * @param subsystem subsystem to be identified as the source of the message
   * @param text text to be logged
   * @return this logger object (for chained operations)
   *
   * @see edu.wpi.first.wpilibj2.command.Subsystem.getName()
   */
  public EventLogger log(Subsystem subsystem, String text) {
    return log(subsystem.getName(), text);
  }

  //////////////////////////////////////////////////////////////////////
  //
  // Abstract methods
  //
  //////////////////////////////////////////////////////////////////////

  /**
   *
   * @param key
   * @param value
   * @return this logger object (for chained operations)
   */
  public abstract EventLogger log(String key, String value);

  /**
   * Object to be logged (as appropriate for the underlying logger: this could be as a String, or
   * serialized data, etc.)
   *
   * @param object object to be logged
   * @return this logger object (for chained operations)
   */
  public abstract EventLogger log(Object data);

  /**
   * Logs a text string.
   *
   * @param text text to be logged
   * @return this logger object (for chained operations)
   */
  public abstract EventLogger logText(String text);

  //////////////////////////////////////////////////////////////////////
  //
  // Implementation of the class
  //
  //////////////////////////////////////////////////////////////////////

  /**
   * Captures events as timestamped (both clock and event) text.
   */
  public static class TextLogger extends EventLogger {
    /**
     * Constructor
     * @param w writer to use as a basis for operations
     */
    public TextLogger(Writer w) {
      super(w);
    }

    /**
     * Generates a prefix for logged data, including timestamps (both calendar and match).
     *
     * @return prefix for logged data
     */
    private String getEventPrefix() {
      Instant now = Instant.now();
      StringBuilder builder = new StringBuilder(now.toString());
      builder.append(" (").append(DriverStation.getMatchTime()).append(")").append(": ");
      return builder.toString();
    }

    @Override
    public synchronized EventLogger logText(String text) {
      try {
        m_output.write(getEventPrefix());
        m_output.write(": ");
        m_output.write(text);
        m_output.write('\n');
        m_output.flush();
      } catch (java.io.IOException ioe) {
        // No-op
      }
      return this;
    }

    @Override
    public EventLogger log(String key, String value) {
      return logText(
          // Stringify the key
          new StringBuilder(key)
              // Add separation
              .append(": ")
              // Add value
              .append(value)
              .toString());
    }

    @Override
    public synchronized EventLogger log(Object data) {
      return logText(data == null ? "null" : data.toString());
    }
  }

  /**
   * TextLogger that will record data to file.
   */
  public static class FileLogger extends TextLogger {
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

  /**
   * TextLogger that will record data to a StringWriter.
   *
   * This is mostly intended for debugging the event logging functionality.
   */
  public static class StringEventLogger extends TextLogger {
    /** Constructor. */
    public StringEventLogger() {
      super(new StringWriter());
      logText("Logging started");
    }

    /**
     * Returns the contents of the log to date.
     * @return the contents of the log to date
     */
    public String getContents() {
      return ((StringWriter) m_output).toString();
    }
  }
}
