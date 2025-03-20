// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Generic interface to a CANdle, allowing both "real" and simulated support. */
public abstract class AbstractCandle extends SubsystemBase {
  static public final Color8Bit ORANGE = new Color8Bit(255, 165, 0);
  static public final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  static public final Color8Bit BLUE = new Color8Bit(0, 0, 255);
  static public final Color8Bit RED = new Color8Bit(255, 0, 0);
  static public final Color8Bit BLACK = new Color8Bit(0, 0, 0);

  /** Constructor. */
  public AbstractCandle() {
    setName("Candle");
  }

  /**
   * Sets the intensity/brightness (as a value from [0-1]).
   * @param value new intensity value
   */
  public abstract void setIntensity(double value);

  /**
   * Sets the color shown on the LEDs.
   * @param color new color value
   */
  public abstract void setColor(Color8Bit color);
}
