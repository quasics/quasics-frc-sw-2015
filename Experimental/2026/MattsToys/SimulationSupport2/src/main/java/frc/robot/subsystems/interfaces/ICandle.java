// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.interfaces.ILighting.StockColor;

/**
 * Simple (trivial!) interface to a CTRE "CANdle" device that can be set to a
 * specific color (e.g., for providing visual signals to the drive team about
 * the robot's status/position/etc.).
 *
 * @see ILighting.StockColor
 * @see <a href="https://store.ctr-electronics.com/products/candle">Product
 *      page</a>
 * @see <a
 *      href=
 *      "https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/CANdle.html">CANdle
 *      class</a>
 */
public interface ICandle extends ISubsystem {
  /** The number of LEDs built into the CANdle hardware. */
  final int CANDLE_DEFAULT_LENGTH = 8;

  /**
   * Sets all of the LEDs on the CANdle to the specified WPI color.
   *
   * @param color the new color for the CANdle's LEDs
   */
  default void setColor(Color color) {
    setColor((int) (color.red * 255) & 0xFF, (int) (color.green * 255) & 0xFF,
        (int) (color.blue * 255) & 0xFF);
  }

  /**
   * Sets all of the LEDs on the CANdle to the specified WPI 8-bit color.
   *
   * @param color the new color for the CANdle's LEDs
   */
  default void setColor(Color8Bit color) {
    setColor(color.red, color.green, color.blue);
  }

  /**
   * Sets all of the LEDs on the CANdle to the specified color.
   *
   * @param color the new color for the CANdle's LEDs
   */
  default void setColor(ILighting.StockColor color) {
    setColor(color.r, color.g, color.b);
  }

  /**
   * Sets all of the LEDs on the CANdle to the specified color.
   *
   * @param r the red component of the new color
   * @param g the green component of the new color
   * @param b the blue component of the new color
   */
  void setColor(int r, int g, int b);

  /**
   * Sets the lighting intensity/brightness.
   *
   * @param intensity new value of lighting brightness [0-1]
   */
  void setIntensity(double intensity);

  /**
   * Helper/sample function to set a candle's lighting, based on the robot's
   * assigned alliance, "enabled" status, and whether or not it is e-stopped.
   *
   * @param candle the ICandle being manipulated
   */
  static void updateCandleForAllianceAndStatus(ICandle candle) {
    StockColor color = StockColor.Green;
    double intensity = 1.0;

    if (!DriverStation.isEnabled()) {
      intensity = 0.25;
    }

    var optAlliance = DriverStation.getAlliance();
    if (optAlliance.isEmpty()) {
      color = StockColor.Green;
    } else {
      var alliance = optAlliance.get();
      color = switch (alliance) {
        case Blue -> StockColor.Blue;
        case Red -> StockColor.Red;
      };
    }

    // Override the color, based on other conditions
    if (DriverStation.isEStopped()) {
      color = StockColor.Orange;
    }

    candle.setIntensity(intensity);
    candle.setColor(color);
  }

  /**
   * A "null object" implementation of the ICandle interface that does nothing.
   * (For example, if we have some robots with an CANdle installed and others
   * without, and don't want to have to check in code every time we might do
   * something with the CANdle to see if we actually have one.)
   */
  public static class NullCandle implements ICandle {
    /** Constructor. */
    public NullCandle() {
      System.out.println("INFO: allocating NullCandle");
    }

    @Override
    public void setColor(int r, int g, int b) {
      // No-op
    }

    @Override
    public void setIntensity(double intensity) {
      // No-op
    }

    @Override
    public void close() throws IOException {
      // No-op
    }
  }
}
