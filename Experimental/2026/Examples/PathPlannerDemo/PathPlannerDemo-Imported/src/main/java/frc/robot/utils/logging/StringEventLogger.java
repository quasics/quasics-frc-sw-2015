package frc.robot.utils.logging;

import java.io.StringWriter;

/**
 * TextLogger that will record data to a StringWriter.
 *
 * This is mostly intended for debugging the event logging functionality.
 */
public class StringEventLogger extends TextLogger {
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