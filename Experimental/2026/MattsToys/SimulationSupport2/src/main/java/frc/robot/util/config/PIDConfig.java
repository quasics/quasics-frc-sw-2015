// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/**
 * PID configuration settings.
 *
 * @param kP proportional constant
 * @param kI integral constant
 * @param kD derivitive constant
 */
public record PIDConfig(double kP, double kI, double kD) {
  /**
   * Overloaded ctor for kP-only configs.
   *
   * @param kP proportional constant
   */
  public PIDConfig(double kP) {
    this(kP, 0.0, 0.0);
  }
}