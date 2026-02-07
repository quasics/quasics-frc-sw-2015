package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {
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

    SmartDashboard.putData(m_name + " Verbosity", m_chooser);

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

  public void log(String out, Verbosity verbosity) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      System.out.println(m_name + " [" + verbosity + "]: " + out);
    }
  }

  public void logError(String out, Verbosity verbosity) {
    if (verbosity.ordinal() >= m_level.ordinal()) {
      System.err.println(m_name + " [" + verbosity + "]: " + out);
    }
  }
}
