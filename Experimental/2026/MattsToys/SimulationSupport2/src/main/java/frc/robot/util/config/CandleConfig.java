package frc.robot.util.config;

/**
 * CANdle configuration settings.
 *
 * @param canId CAN ID for the device; if negative, the device should be
 *              simulated
 */
public record CandleConfig(int canId) {
  /**
   * Determines if the CANdle is simulated.
   *
   * @return true iff the CANdle is simulated (based on the CAN ID)
   */
  public boolean simulated() {
    return canId < 0;
  }
}