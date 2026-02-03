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

  // Annoyingly, need to query m_chooser every time for get selected
  private String m_name;
  private final SendableChooser<Verbosity> m_chooser = new SendableChooser<>();

  public Logger(Verbosity verbosity, String name) {
    m_name = name;

    SmartDashboard.putData(m_name + " Verbosity", m_chooser);

    m_chooser.setDefaultOption(toString(verbosity), verbosity);
    m_chooser.addOption("Debug", Verbosity.Debug);
    m_chooser.addOption("Info", Verbosity.Info);
    m_chooser.addOption("Notice", Verbosity.Notice);
    m_chooser.addOption("Warn", Verbosity.Warn);
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
    if (verbosity.ordinal() >= m_chooser.getSelected().ordinal()) {
      System.out.println(m_name + " [" + verbosity + "]: " + out);
    }
  }
}
