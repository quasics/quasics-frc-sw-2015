package frc.robot.util.config;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

/**
 * Arm Feed forward settings.
 *
 * @param kS static gain
 * @param kG gravity gain
 * @param kV kV, in V/(m/s)
 * @param kA kA, in V/(m/s^2)
 */
public record
    ArmFeedForwardConfig(Voltage kS, Voltage kG, double kV, double kA) {
  /**
   * Overloaded constructor (no kA).
   *
   * @param kS static gain, in V
   * @param kG gravity gain, in V
   * @param kV kV, in V/(m/s)
   */
  public ArmFeedForwardConfig(Voltage kS, Voltage kG, double kV) {
    this(kS, kG, kV, 0);
  }

  /**
   * Overloaded constructor (all unitless).
   *
   * @param kS static gain, in V
   * @param kG gravity gain, in V
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public ArmFeedForwardConfig(double kS, double kG, double kV, double kA) {
    this(Volts.of(kS), Volts.of(kG), kV, kA);
  }

  /**
   * Overloaded constructor (all unitless, no kA).
   *
   * @param kS static gain, in V
   * @param kG gravity gain, in V
   * @param kV kV, in V/(m/s)
   */
  public ArmFeedForwardConfig(double kS, double kG, double kV) {
    this(Volts.of(kS), Volts.of(kG), kV, 0);
  }
}