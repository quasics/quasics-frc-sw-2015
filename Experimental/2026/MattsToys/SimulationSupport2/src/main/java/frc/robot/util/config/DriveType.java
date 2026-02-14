// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/** Drive hardware type (simulated, CAN-based SparkMax, etc.). */
public enum DriveType {
  /**
   * Drive hardware is simulated. (This may include simulation of physical
   * hardware, or be "pure math".)
   */
  Simulated,
  /** Drive hardware is a CAN-based SparkMax motor controller. */
  CanSparkMax,
  /** Drive hardware is a ThriftyNova motor controller. */
  ThriftyNova
}