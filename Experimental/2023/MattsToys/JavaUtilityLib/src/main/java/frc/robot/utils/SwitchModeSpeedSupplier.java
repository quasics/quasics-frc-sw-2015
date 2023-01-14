// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

/** Add your docs here. */
public class SwitchModeSpeedSupplier {
  /** Used to determine if the robot is currently in "switched mode". */
  final Supplier<Boolean> m_directionsSwitchedSupplier;

  /** Supplies speed for left side of robot in "normal" mode. */
  final Supplier<Double> m_leftStickSupplier;
  /** Supplies speed for right side of robot in "normal" mode. */
  final Supplier<Double> m_rightStickSupplier;

  public SwitchModeSpeedSupplier(Supplier<Double> rawLeftStickSupplier,
      Supplier<Double> rawRightStickSupplier, Supplier<Boolean> directionsSwitchedSupplier) {
    m_leftStickSupplier = rawLeftStickSupplier;
    m_rightStickSupplier = rawRightStickSupplier;
    m_directionsSwitchedSupplier = directionsSwitchedSupplier;
  }

  public Supplier<Double> getLeftSpeedSupplier() {
    return () -> {
      if (m_directionsSwitchedSupplier.get()) {
        return -m_rightStickSupplier.get();
      }
      return m_leftStickSupplier.get();
    };
  }

  public Supplier<Double> getRightSpeedSupplier() {
    return () -> {
      if (m_directionsSwitchedSupplier.get()) {
        return -m_leftStickSupplier.get();
      }
      return m_rightStickSupplier.get();
    };
  }
}
