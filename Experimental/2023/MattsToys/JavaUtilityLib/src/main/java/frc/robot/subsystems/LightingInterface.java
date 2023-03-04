package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

public interface LightingInterface {
  /** Interface to use in populating the data for each LED on the strip. */
  public interface ColorSupplier {
    /** Returns the color to be used for the LED at a given position on the strip. */
    public Color getColorForLed(int position);
  }

  /**
   * Primary (most flexible) mechanism for controlling the lights on the robot.
   *
   * @param function used to generate the color for each position on the set of lights.
   */
  public void SetStripColor(ColorSupplier function);

  /** Convenience type, used to provide (named) values for different LED colors. */
  public enum StockColor {
    Green(0, 255, 0),
    Red(255, 0, 0),
    Blue(0, 0, 255),
    White(255, 255, 255),
    Black(0, 0, 0);

    /** RGB color values (ranging from 0-255 for each component). */
    public final int r, g, b;

    /** Constructor */
    StockColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }

    /** Converts the stock color to the representation used by WPILib. */
    public Color toWpiColor() {
      return toWpiColor(1.0);
    }

    /**
     * Converts the stock color to the representation used by WPILib, scaled to the specified
     * intensity (brightness).
     *
     * @param intensityPercent intensity to which the color should be scaled (0.0-1.0)
     */
    public Color toWpiColor(double intensityPercent) {
      return new Color(
          intensityPercent * r / 255.0, intensityPercent * g / 255.0, intensityPercent * b / 255.0);
    }
  }

  /**
   * Convenience function: sets all contolled lights to a solid color from the stock set.
   *
   * @param color the color to make all of the lights in the strip
   */
  public default void SetStripColor(StockColor color) {
    SetStripColor(color, 1.0);
  }

  /**
   * Convenience function: sets all contolled lights to a solid color from the stock set, scaled to
   * the specified % intensity.
   *
   * @param color the color to make all of the lights in the strip
   * @param intensityPercent intensity to which the color should be scaled (0.0-1.0)
   */
  public default void SetStripColor(StockColor color, double intensityPercent) {
    SetStripColor(color.toWpiColor(intensityPercent));
  }

  /**
   * Convenience function: sets all controlled lights to a solid color, specified as an RGB tripet
   * of values (each [0..255]).
   *
   * @param red red component (0-255)
   * @param green green component (0-255)
   * @param blue blue component (0-255)
   */
  public default void SetStripColor(int red, int green, int blue) {
    // Note: WPI expects color component values to be percentages in the range
    // [0.0-1.0], so we need to convert to the right scale.
    var color = new Color(red / 255.0, green / 255.0, blue / 255.0);

    SetStripColor(color);
  }

  /**
   * Convenience function: sets all controlled lights to a solid color.
   *
   * <p>Note: unlike with various other APIs, the component values for WPI's version of color
   * specification are all as percentages (0.0-1.0).
   *
   * @param color the color to make all of the lights in the strip
   */
  public default void SetStripColor(Color color) {
    // Defines a "lambda" function that will be used to fulfill the
    // requirements of the ColorFunctor type. (It will return the
    // same color for each position in the strip.)
    ColorSupplier function = (var position) -> color;

    // Uses the lambda to set the color for the full strip.
    SetStripColor(function);
  }

  /**
   * Trivial implementation of the interface, for use on robots that don't support lighting (but
   * want to have a subsystem available for convenience/common code).
   */
  public static final class MockLighting implements LightingInterface {
    @Override
    public void SetStripColor(ColorSupplier function) {
      // Do nothing with the lighting request.
    }
  }
}
