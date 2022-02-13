// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
  public enum StockColor {
    Green(0, 255, 0),
    Red(255, 0, 0),
    Blue(0, 0, 255),
    White(255, 255, 255),
    Black(0, 0, 0);

    final private int r, g, b;

    // Constructor
    StockColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }

    /**
     * Converts the stock color to the representation used by WPILib.
     */
    public Color toWpiColor() {
      return toWpiColor(1.0);
    }

    /**
     * Converts the stock color to the representation used by WPILib, scaled to the
     * specified intensity (brightness).
     * 
     * @param intensityPercent intensity to which the color should be scaled
     *                         (0.0-1.0)
     */
    public Color toWpiColor(double intensityPercent) {
      return new Color(
          intensityPercent * r / 255.0,
          intensityPercent * g / 255.0,
          intensityPercent * b / 255.0);
    }

    /** Returns the red component of the color (0..255). */
    public int getR() {
      return r;
    }

    /** Returns the green component of the color (0..255). */
    public int getG() {
      return g;
    }

    /** Returns the blue component of the color (0..255). */
    public int getB() {
      return b;
    }
  }

  /** The raw interface to the addressable LED strip connected to the Rio. */
  final private AddressableLED m_led;
  /** The buffer used to set the values for each pixel/LED on the strip. */
  final private AddressableLEDBuffer m_ledBuffer;

  /**
   * Constructor.
   * 
   * @param pwmPort   PWM port to which the LED strip is connected
   * @param numLights number of (logical) lights on the LED strip
   */
  public Lighting(int pwmPort, int numLights) {
    m_led = new AddressableLED(pwmPort);

    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led.setLength(m_ledBuffer.getLength());

    // On start-up, turn every other pixel on (white).
    if (true) {
      var white = StockColor.White.toWpiColor();
      var black = StockColor.Black.toWpiColor();
      ColorFunctor function = (int position) -> {
        return (position % 2 == 0) ? white : black;
      };
      SetStripColor(function);
    } else {
      // TODO(mjh): Remove this clause, once the other path has been tested.
      final var fixedColor = new edu.wpi.first.wpilibj.util.Color(255, 255, 255);
      for (var i = 0; i < m_ledBuffer.getLength(); i += 2) {
        m_ledBuffer.setLED(i, fixedColor);
      }
      m_led.setData(m_ledBuffer);
    }

    // Start up the LED handling.
    m_led.start();
  }

  /** Helper interface to use in populating the data for each LED on the strip. */
  public interface ColorFunctor {
    /**
     * Returns the color to be used for the LED at a given position on the strip.
     */
    public Color getColorForLed(int position);
  }

  /**
   * Sets the color for each LED in the strip, using the specified function to
   * generate the values for each position.
   * 
   * @param function Function generating the color for each LED
   */
  public void SetStripColor(ColorFunctor function) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, function.getColorForLed(i));
    }

    m_led.setData(m_ledBuffer);
  }

  /**
   * Convenience function: sets the strip to a solid color.
   * 
   * Note: unlike with various other APIs, the component values for WPI's version
   * of color specification are all as percentages (0.0-1.0).
   * 
   * @param color the color to make all of the lights in the strip
   */
  public void SetStripColor(Color color) {
    // Defines a "lambda" function that will be used to fulfill the
    // requirements of the ColorFunctor type. (It will return the
    // same color for each position in the strip.)
    ColorFunctor function = (var position) -> color;

    // Uses the lambda to set the color for the full strip.
    SetStripColor(function);
  }

  /**
   * Convenience function: sets the strip to a solid color from the stock set.
   * 
   * @param color the color to make all of the lights in the strip
   */
  public void SetStripColor(StockColor color) {
    SetStripColor(color.toWpiColor());
  }

  /**
   * Convenience function: sets the strip to a solid color, specified as an RGB
   * tripet of values (each [0..255]).
   * 
   * @param red   red component (0-255)
   * @param green green component (0-255)
   * @param blue  blue component (0-255)
   */
  public void SetStripColor(int red, int green, int blue) {
    // Note: WPI expects color component values to be percentages in the range
    // [0.0-1.0], so we need to convert to the right scale.
    var color = new Color(
        red / 255.0,
        green / 255.0,
        blue / 255.0);

    SetStripColor(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
