// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

/**
 * Simple (trivial!) interface to a CTRE "CANdle" device that can be set to a
 * specific color.
 * 
 * @see <a href="https://store.ctr-electronics.com/products/candle">Product
 *      page</a>
 * @see <a
 *      href=
 *      "https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html">CANdle
 *      class
 * @see ILighting.StockColor
 */
public interface ICandle extends ISubsystem {

  /** The number of LEDs built into the CANdle hardware. */
  final int CANDLE_DEFAULT_LENGTH = 8;

  /** Sets all of the LEDs on the CANdle to the specified color. */
  default void setColor(ILighting.StockColor color) {
    setColor(color.r, color.g, color.b);
  }

  /** Sets all of the LEDs on the CANdle to the specified color. */
  void setColor(int r, int g, int b);

  public static class NullCandle implements ICandle {
    @Override
    public void setColor(int r, int g, int b) {
      // Do nothing.
    }
  }
}
