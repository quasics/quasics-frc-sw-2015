// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.time.Instant;

/** Add your docs here. */
public abstract class EventLogger {
  final protected Writer m_output;

  protected EventLogger(Writer output) {
    m_output = output;
  }

  /** A convenience object, which will handle all writing as no-ops. */
  public static final Writer NULL_WRITER = new Writer() {
    @Override
    public void write(char[] cbuf, int off, int len) throws IOException {
      // No-op
    }

    @Override
    public void flush() throws IOException {
      // No-op
    }

    @Override
    public void close() throws IOException {
      // No-op
    }
  };

  //////////////////////////////////////////////////////////////////////
  //
  // Abstract methods
  //
  //////////////////////////////////////////////////////////////////////

  public abstract EventLogger log(String key, String value);
  public abstract EventLogger log(Object data);
  public abstract EventLogger logText(String text);

  //////////////////////////////////////////////////////////////////////
  //
  // Implementations
  //
  //////////////////////////////////////////////////////////////////////

  /**
   * Captures events as timestamped (both clock and event) text.
   */
  public static class TextLogger extends EventLogger {
    public TextLogger(Writer w) {
      super(w);
    }

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
    public FileLogger(File f) throws java.io.IOException {
      super(new FileWriter(f));
    }

    public FileLogger(File f, Object noThrow) throws java.io.IOException {
      super(new FileWriter(f));
    }

    static Writer getWriterNoThrow(File f) {
      try {
        return new FileWriter(f);
      } catch (java.io.IOException ioe) {
        return NULL_WRITER;
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
