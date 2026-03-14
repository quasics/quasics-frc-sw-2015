// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Simple logging utility, including volume control (which can be altered either
 * programmatically or via a selector on the dashboard).
 */
public class Logger {
  /** Logging levels, controlling output. */
  public enum Level {
    /** Critical failure. */
    Critical,
    /** Error. */
    Error,
    /** Warning. */
    Warning,
    /** Informational. */
    Info,
    /** Debugging output. */
    Debug,
    ;
  }

  /** Name associated with the logger. */
  final String m_name;

  /**
   * Current logging (output) threshold: anything less important than this will
   * not be logged.
   */
  Level m_level;

  /**
   * The dashboard selector that can be used to interactively change the threshold
   * level for this logger.
   */
  final SendableChooser<Level> m_levelChooser;

  /**
   * Constructor.
   * 
   * @param name  name associated with the logger; used as a prefix on output, and
   *              also used to label the chooser providing "volume control"
   * @param level initial logging threshold
   */
  public Logger(String name, Level level) {
    m_name = name;
    m_level = level;

    m_levelChooser = new SendableChooser<Level>();
    for (var l : Level.values()) {
      m_levelChooser.addOption(l.name(), l);
    }
    m_levelChooser.setDefaultOption(level.name(), level);

    DashboardUtils.publish("Logging", getDashboardLabel(), m_levelChooser);
    m_levelChooser.onChange(this::loggingLevelChanged);
  }

  /**
   * Returns the label associated with this logger's "volume control" on the
   * dashboard.
   */
  private String getDashboardLabel() {
    return m_name + " verbosity";
  }

  /**
   * Event handler, processing changes to the verbosity selector on the dashboard.
   * 
   * @param level newly-selected logging level
   */
  private void loggingLevelChanged(Level level) {
    m_level = level;
  }

  /**
   * Updates the selected verbosity level for this logger.
   * 
   * Note that this is also posting it to the sendable selector, so that the level
   * shown in the UX reflects the programmatic level change.
   * 
   * @param level new logging level to be applied
   */
  public void setLevel(Level level) {
    var table = DashboardUtils.getNetworkTable("Logging", getDashboardLabel());
    table.getEntry("selected").setString(level.name());

    m_level = level;
  }

  /**
   * Logs the specified output, if the output level is above the current threshold
   * for the logger.
   * 
   * @param level  verbosity associated with this log message
   * @param output text to be logged
   */
  public void log(Level level, String output) {
    if (level.ordinal() <= m_level.ordinal()) {
      System.out.format("%s [%s] %s\n", m_name, level.name(), output);
    }
  }

  /**
   * Logs the specified formatted output, if the output level is above the current
   * threshold for the logger.
   * 
   * @param level  verbosity associated with this log event
   * @param format a format string
   * @param args   aguments referenced by the format specifiers in the format
   *               string
   * 
   * @see java.lang.String#format(String, Object...)
   */
  public void logFormatted(Level level, String format, Object... args) {
    log(level, String.format(format, args));
  }
}
