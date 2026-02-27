// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.NullWriter;
import java.io.Writer;

/**
 * Defines an abstract type to support logging operations/events on the robot.
 */
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
   * Convenience logging function.
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
}
