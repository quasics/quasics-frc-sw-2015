// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Implements a reusable "add-on" for handling speed scaling
 * (normal/turbo/turtle) for robot driving.
 */
public class SpeedModeScaler implements UnaryOperator<Double> {
  /** Supplies the current "driving mode" for the robot. */
  final Supplier<SpeedMode> m_modeSupplier;

  /** Scaling factor applied in "normal" mode. */
  final double m_normalScaling;

  /** Scaling factor applied in "turbo" mode. */
  final double m_turboScaling;

  /** Scaling factor applied in "turtle" mode. */
  final double m_turtleScaling;

  /**
   * Constructor.
   *
   * @param modeSupplier  supplies the current "driving mode" for the robot
   * @param normalScaling scaling factor applied in "normal" mode
   * @param turboScaling  scaling factor applied in "turbo" mode
   * @param turtleScaling scaling factor applied in "turtle" mode
   *
   * @throws IllegalArgumentException if normalScaling > turboScaling, or
   *                                  turtleScaling > normalScaling
   */
  public SpeedModeScaler(Supplier<SpeedMode> modeSupplier, double normalScaling,
      double turboScaling, double turtleScaling) {
    if (modeSupplier == null) {
      throw new IllegalArgumentException("modeSupplier cannot be null");
    }
    if (turtleScaling > normalScaling) {
      throw new IllegalArgumentException(
          "turtleScaling must be <= normalScaling");
    }
    if (normalScaling > turboScaling) {
      throw new IllegalArgumentException(
          "normalScaling must be <= turboScaling");
    }
    m_modeSupplier = modeSupplier;
    m_normalScaling = normalScaling;
    m_turboScaling = turboScaling;
    m_turtleScaling = turtleScaling;
  }

  @Override
  public Double apply(Double t) {
    return switch (m_modeSupplier.get()) {
      case Normal -> t* m_normalScaling;
      case Turbo -> t* m_turboScaling;
      case Turtle -> t* m_turtleScaling;
    };
  }
}
