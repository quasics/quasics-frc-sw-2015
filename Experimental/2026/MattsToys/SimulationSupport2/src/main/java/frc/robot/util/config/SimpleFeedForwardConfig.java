// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

/**
 * Simple (i.e., not for elevators) feedforward data.
 *
 * @param kS kS, in V
 * @param kV kV, in V/(m/s); must be > 0
 * @param kA kA, in V/(m/s^2)
 */
public record SimpleFeedForwardConfig(Voltage kS, Voltage kV, double kA) {
  /**
   * Overloaded constructor.
   *
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public SimpleFeedForwardConfig(Voltage kV, double kA) {
    this(Volts.of(0), kV, kA);
  }

  /**
   * Overloaded constructor.
   *
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public SimpleFeedForwardConfig(double kV, double kA) {
    this(Volts.of(0), Volts.of(kV), kA);
  }

  /**
   * Overloaded constructor.
   *
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public SimpleFeedForwardConfig(Voltage kS, double kV, double kA) {
    this(kS, Volts.of(kV), kA);
  }
}