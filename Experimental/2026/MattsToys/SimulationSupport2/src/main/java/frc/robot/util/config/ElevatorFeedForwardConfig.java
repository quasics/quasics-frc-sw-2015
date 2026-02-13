// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

/**
 * Elevator Feed forward settings.
 *
 * @param kS static gain
 * @param kG gravity gain
 * @param kV kV, in V/(m/s)
 * @param kA kA, in V/(m/s^2)
 */
public record ElevatorFeedForwardConfig(
    Voltage kS, Voltage kG, double kV, double kA) {
  /**
   * Overloaded constructor.
   *
   * @param kS static gain, in V
   * @param kG gravity gain, in V
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public ElevatorFeedForwardConfig(
      double kS, double kG, double kV, double kA) {
    this(Volts.of(kS), Volts.of(kG), kV, kA);
  }
}