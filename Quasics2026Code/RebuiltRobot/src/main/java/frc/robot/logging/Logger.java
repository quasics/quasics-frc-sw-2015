package frc.robot.logging;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simple logging class with configurable "volume".
 */
public class Logger {
  /**
   * Controls if "volume selectors" will be put on a dedicated tab in the
   * dashboard (true), or if they will go on the main tab (false).
   */
  private final static boolean USE_SUB_PAGE_FOR_CHOOSERS = false;

  /**
   * Logging level associated with a message, and for the threshold associated
   * Generally want to set verbosity per-subsystem
   */
  public enum Verbosity {
    /** Super duper debug - often want to set per-subsystem. */
    Debug,
    /** Some debug spew but readable. */
    Info,
    /** No spew/very readable [Default]. */
    Notice,
    /** Only things which indicate problems. */
    Warn
  }

  /**
   * Name associated with this logger. (Included in messages and on the label for
   * the "volume selector" in the dashboard.)
   */
  private String m_name;

  /** Volume selector for this logger, shown on the dashboard. */
  private final SendableChooser<Verbosity> m_chooser = new SendableChooser<>();

  /**
   * Threshold volume associated with the logger: any messages less important than
   * this will not be logged.
   */
  Verbosity m_level = Verbosity.Notice;

  /**
   * Constructs a Logger using a default volume threshold of "Notice".
   * 
   * @param name name associated with the logger
   */
  public Logger(String name) {
    this(Verbosity.Notice, name);
  }

  /**
   * Constructs a Logger with the specified volume threshold.
   * 
   * @param verbosity initial volume threshold for the logger
   * @param name      name associated with the logger
   */
  public Logger(Verbosity verbosity, String name) {
    m_name = name;
    m_level = verbosity;

    if (USE_SUB_PAGE_FOR_CHOOSERS) {
      var loggingTab = Shuffleboard.getTab("Logging");
      loggingTab.add(m_name + " Verbosity", m_chooser);
    } else {
      SmartDashboard.putData(m_name + " Verbosity", m_chooser);
    }

    for (var volume : Verbosity.values()) {
      m_chooser.addOption(volume.name(), volume);
    }
    m_chooser.setDefaultOption(verbosity.name(), verbosity);

    m_chooser.onChange(this::setVerbosity);
  }

  /**
   * Callback function, used to update the logging threshold when it is changed on
   * the dashboard.
   * 
   * @param level new logging threshold
   */
  private void setVerbosity(Verbosity level) {
    m_level = level;
  }

  /**
   * Logs the specified message.
   * 
   * @param verbosity priority associated with the message
   * @param out       message to be logged
   */
  public void log(Verbosity verbosity, String out) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      System.out.println(m_name + " [" + verbosity + "]: " + out);
    }
  }

  /**
   * Logs the specified message.
   * 
   * @param verbosity priority associated with the message
   * @param out       message to be logged
   */
  public void logError(Verbosity verbosity, String out) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      System.err.println(m_name + " [" + verbosity + "]: " + out);
    }
  }

  /**
   * Convenience function for handling formatted output (without having to
   * actually apply formatting if the output won't get logged).
   * 
   * @param verbosity output level associated with this message
   * @param format    formatting string to be used
   * @param args      objects to be applied to the formatting string
   * 
   * @see java.lang.String#format(String, Object...)
   */
  public void logFormatted(Verbosity verbosity, String format, Object... args) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      log(verbosity, String.format(format, args));
    }
  }
}
