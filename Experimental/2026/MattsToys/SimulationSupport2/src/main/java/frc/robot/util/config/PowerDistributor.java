// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Power distribution configuration settings.
 *
 * @param type  power distribution panel type
 * @param canId CAN ID for the device
 */
public record PowerDistributor(
    PowerDistribution.ModuleType type, int canId) {
  /**
   * Constructor, which will set the CAN ID based on the default for the
   * specified module type.
   *
   * @param type power distribution panel type
   */
  public PowerDistributor(PowerDistribution.ModuleType type) {
    this(type, switch (type) {
      case kCTRE -> 0;
      case kRev -> 1;
    });
  }
}