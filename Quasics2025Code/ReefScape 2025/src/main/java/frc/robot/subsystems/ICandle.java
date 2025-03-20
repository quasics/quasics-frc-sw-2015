// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Generic interface to a CANdle, allowing both "real" and simulated support. */
public interface ICandle {
  /**
   * Sets the intensity/brightness (as a value from [0-1]).
   * @param value new intensity value
   */
  void setIntensity(double value);

  /**
   * Sets the color shown on the LEDs.
   * @param color new color value
   */
  void setColor(Color8Bit color);
}
