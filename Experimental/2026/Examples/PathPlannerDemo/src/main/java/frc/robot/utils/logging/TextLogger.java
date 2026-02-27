package frc.robot.utils.logging;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.Writer;
import java.time.Instant;

/**
 * Captures events as timestamped (both clock and event) text.
 */
public class TextLogger extends EventLogger {
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