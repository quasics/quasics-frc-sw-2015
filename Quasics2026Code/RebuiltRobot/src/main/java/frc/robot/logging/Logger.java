package frc.robot.logging;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
  private final static boolean USE_SUB_PAGE_FOR_CHOOSERS = false;

  // Generally want to set verbosity per-subsystem
  public enum Verbosity {
    Debug, // Super duper debug - often want to set per-subsystem
    Info, // Some debug spew but readable
    Notice, // No spew/very readable [Default]
    Warn // Only things which indicate problems
  }

  private String m_name;
  private final SendableChooser<Verbosity> m_chooser = new SendableChooser<>();
  Verbosity m_level = Verbosity.Notice;

  public Logger(Verbosity verbosity, String name) {
    m_name = name;
    m_level = verbosity;

    if (USE_SUB_PAGE_FOR_CHOOSERS) {
      var loggingTab = Shuffleboard.getTab("Logging");
      loggingTab.add(m_name + " Verbosity", m_chooser);
    } else {
      SmartDashboard.putData(m_name + " Verbosity", m_chooser);
    }

    m_chooser.setDefaultOption(toString(verbosity), verbosity);
    m_chooser.addOption("Debug", Verbosity.Debug);
    m_chooser.addOption("Info", Verbosity.Info);
    m_chooser.addOption("Notice", Verbosity.Notice);
    m_chooser.addOption("Warn", Verbosity.Warn);

    m_chooser.onChange(this::setVerbosity);
  }

  private void setVerbosity(Verbosity level) {
    m_level = level;
  }

  public String toString(Verbosity verbosity) {
    switch (verbosity) {
      case Debug:
        return "Debug";
      case Info:
        return "Info";
      case Notice:
        return "Notice";
      case Warn:
        return "Warn";

      default:
        return "Unknown";
    }
  }

  public void log(Verbosity verbosity, String out) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      System.out.println(m_name + " [" + verbosity + "]: " + out);
    }
  }

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
