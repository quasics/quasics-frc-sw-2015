// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/**
 * A simple record to hold the configuration for a flywheel subsystem. This can
 * be used to easily create different types of flywheel subsystems based on the
 * configuration provided (e.g., SparkMax, TalonFX, etc.).
 */
public record FlywheelConfig(FlywheelType type, int motorID, boolean inverted) {
  public enum FlywheelType {
    SparkMax,
    TalonFX,
    Null
  }
}
