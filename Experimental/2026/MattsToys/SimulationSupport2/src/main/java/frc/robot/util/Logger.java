package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Logger {
  /** Logging levels, controlling output. */
  public enum Verbosity {
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

  final String m_label;

  /**
   * Current logging (output) threshold: anything less important than this will
   * not be logged.
   */
  Verbosity m_level;

  final SendableChooser<Verbosity> m_levelChooser;

  /**
   * Constructor.
   * 
   * @param name  name associated with the logger; used as a prefix on output, and
   *              also used to label the chooser providing "volume control"
   * @param level initial logging threshold
   */
  public Logger(String name, Verbosity level) {
    m_name = name;
    m_level = level;

    m_levelChooser = new SendableChooser<Verbosity>();
    for (var l : Verbosity.values()) {
      m_levelChooser.addOption(l.name(), l);
    }
    m_levelChooser.setDefaultOption(level.name(), level);

    m_label = name + " verbosity";

    DashboardUtils.publish("Logging", m_label, m_levelChooser);
    m_levelChooser.onChange(this::loggingLevelChanged);
  }

  /**
   * Event handler, processing changes to the verbosity selector on the dashboard.
   * 
   * @param level newly-selected logging level
   */
  private void loggingLevelChanged(Verbosity level) {
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
  public void setLevel(Verbosity level) {
    var table = DashboardUtils.getNetworkTable("Logging", m_label);
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
  public void log(Verbosity level, String output) {
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
  public void logFormatted(Verbosity level, String format, Object... args) {
    log(level, String.format(format, args));
  }
}
