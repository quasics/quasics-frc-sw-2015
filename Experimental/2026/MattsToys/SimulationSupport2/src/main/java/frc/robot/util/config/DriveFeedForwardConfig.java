package frc.robot.util.config;

import edu.wpi.first.units.measure.Voltage;

/**
 * Drive Feed forward settings.
 *
 * @param linear  linear feedforward settings
 * @param angular angular (rotational) feedforward settings
 */
public record DriveFeedForwardConfig(
    SimpleFeedForwardConfig linear, SimpleFeedForwardConfig angular) {
  /**
   * Overloaded constructor.
   *
   * @para, ksLinear linear feedforward kS value
   *
   * @param kvLinear  linear feedforward kV value
   * @param kaLinear  linear feedforward kA value
   * @param kvAngular angular (rotational) feedforward kV value
   * @param kaAngular angular (rotational) feedforward kA value
   */
  public DriveFeedForwardConfig(Voltage ksLinear, Voltage kvLinear,
      double kaLinear, Voltage kvAngular, double kaAngular) {
    this(new SimpleFeedForwardConfig(ksLinear, kvLinear, kaLinear),
        new SimpleFeedForwardConfig(kvAngular, kaAngular));
  }

  /**
   * Overloaded constructor, taking pairs of (kV, kA) as discrete values
   *
   * @param kvLinear  linear feedforward kV value
   * @param kaLinear  linear feedforward kA value
   * @param kvAngular angular (rotational) feedforward kV value
   * @param kaAngular angular (rotational) feedforward kA value
   */
  public DriveFeedForwardConfig(
      Voltage kvLinear, double kaLinear, Voltage kvAngular, double kaAngular) {
    this(new SimpleFeedForwardConfig(kvLinear, kaLinear),
        new SimpleFeedForwardConfig(kvAngular, kaAngular));
  }
}