// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

/** Add your docs here. */
public class SwitchModeSpeedSupplier {
  /** Supplies speed for left side of robot. */
  final Supplier<Double> m_leftSpeedSupplier;

  /** Supplies speed for right side of robot. */
  final Supplier<Double> m_rightSpeedSupplier;

  /**
   * Used to store if "switch mode" is engaged (i.e., if we're now treating the rear of the robot as
   * the front).
   */
  private boolean m_switchModeEngaged = false;

  /**
   * Constructor.
   * 
   * @param leftStickSupplier used to get current value of left driver joystick
   * @param rightStickSupplier used to get current value of right driver joystick
   */
  public SwitchModeSpeedSupplier(Supplier<Double> leftStickSupplier,
      Supplier<Double> rightStickSupplier) {
    m_leftSpeedSupplier = () -> {
      if (m_switchModeEngaged) {
        return -rightStickSupplier.get();
      }
      return leftStickSupplier.get();
    };

    m_rightSpeedSupplier = () -> {
      if (m_switchModeEngaged) {
        return -leftStickSupplier.get();
      }
      return rightStickSupplier.get();
    };
  }

  /**
   * Use this to toggle "switch mode" (from engaged -> not, and vice versa).
   */
  public void toggleSwitchMode() {
    m_switchModeEngaged = !m_switchModeEngaged;
  }

  /**
   * Use this to determine if "switch mode" is engaged (i.e., if we're now treating the rear of the
   * robot as the front).
   * 
   * @return true iff "switch mode" is engaged
   */
  public boolean switchModeEngaged() {
    return m_switchModeEngaged;
  }

  /**
   * Returns a SpeedSupplier for the left-hand side motors that will respect the configured state
   * for the switch mode.
   * 
   * @return SpeedSupplier for the left-hand side motors
   */
  public Supplier<Double> getLeftSpeedSupplier() {
    return m_leftSpeedSupplier;
  }

  /**
   * Returns a SpeedSupplier for the right-hand side motors that will respect the configured state
   * for the switch mode.
   * 
   * @return SpeedSupplier for the right-hand side motors
   */
  public Supplier<Double> getRightSpeedSupplier() {
    return m_rightSpeedSupplier;
  }
}
