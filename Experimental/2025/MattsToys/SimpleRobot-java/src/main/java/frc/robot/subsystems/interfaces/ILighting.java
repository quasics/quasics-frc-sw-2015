// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Basic interface to a lighting subsystem.
 */
public interface ILighting extends ISubsystem {
  /** Interface to use in populating the data for each LED on the strip. */
  public interface ColorSupplier {
    /**
     * @param position the position of an LED on the strip
     * @return the color to be used for the LED
     */
    public Color getColorForLed(int position);
  }

  /**
   * Primary (most flexible) mechanism for controlling the lights on the robot.
   *
   * @param function used to generate the color for each position on the set of
   *                 lights.
   */
  public void SetStripColor(ColorSupplier function);

  /**
   * Convenience type, used to provide (named) values for different LED colors.
   */
  public enum StockColor {
    /** White. */
    White(255, 255, 255),
    /** Black (off). */
    Black(0, 0, 0),
    /** Green. (Actually "lime green".) */
    Green(0, 255, 0),
    /** Red. */
    Red(255, 0, 0),
    /** Blue. */
    Blue(0, 0, 255),
    /** Fuchsia. */
    Fuchsia(255, 0, 255),
    /** Aqua. */
    Aqua(0, 255, 255),
    /** Gray. */
    Gray(128, 128, 128),
    /** Maroon. */
    Maroon(128, 0, 0),
    /** Dark green. */
    DarkGreen(0, 128, 0),
    /** Navy. */
    Navy(0, 0, 128),
    /** Olive. */
    Olive(128, 128, 0),
    /** Purple. */
    Purple(128, 0, 128),
    /** Silver. */
    Silver(0xC0, 0xC0, 0xC0),
    /** Teal. */
    Teal(0, 128, 128),
    /** Orange. */
    Orange(0xFF, 0x78, 0x02),
    /** Pink. (Specifically, "deep pink".) */
    Pink(0xFF, 0x14, 0x93),
    /** Gold. */
    Gold(0xFF, 0xD7, 0x00),
    /** Yellow. */
    Yellow(0xFF, 0xFF, 0x00),
    /** White smoke. */
    WhiteSmoke(0xF5, 0xF5, 0xF5);

    /** RGB color values (ranging from 0-255 for each component). */
    public final int r, g, b;

    /**
     * Constructor.
     *
     * @param r red component
     * @param g green component
     * @param b blue component
     */
    StockColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }

    /** @return the representation of this color used by WPILib. */
    public Color toWpiColor() {
      return toWpiColor(1.0);
    }

    /**
     * Converts the stock color to the representation used by WPILib, scaled to the
     * specified intensity (brightness).
     *
     * @param intensityPercent intensity to which the color should be scaled
     *                         (0.0-1.0)
     * @return the scaled value of this color, as used by WPILib
     */
    public Color toWpiColor(double intensityPercent) {
      return new Color(
          intensityPercent * r / 255.0, intensityPercent * g / 255.0, intensityPercent * b / 255.0);
    }
  }

  /**
   * Convenience function: sets all contolled lights to a solid color from the
   * stock set.
   *
   * @param color the color to make all of the lights in the strip
   */
  public default void SetStripColor(StockColor color) {
    SetStripColor(color, 1.0);
  }

  /**
   * Convenience function: sets all contolled lights to a solid color from the
   * stock set, scaled to
   * the specified % intensity.
   *
   * @param color            the color to make all of the lights in the strip
   * @param intensityPercent intensity to which the color should be scaled
   *                         (0.0-1.0)
   */
  public default void SetStripColor(StockColor color, double intensityPercent) {
    SetStripColor(color.toWpiColor(intensityPercent));
  }

  /**
   * Convenience function: sets all controlled lights to a solid color, specified
   * as an RGB tripet
   * of values (each [0..255]).
   *
   * @param red   red component (0-255)
   * @param green green component (0-255)
   * @param blue  blue component (0-255)
   */
  public default void SetStripColor(int red, int green, int blue) {
    // Note: WPI expects color component values to be percentages in the range
    // [0.0-1.0], so we need to convert to the right scale.
    var color = new Color(red / 255.0, green / 255.0, blue / 255.0);

    SetStripColor(color);
  }

  /**
   * Sets the strip to an alternating pair of colors, with even pixels set to
   * color1, and odd pixels set to color2.
   * 
   * @param color1 color to use for even pixels (starting at 0)
   * @param color2 color to use for odd pixels (starting at 1)
   */
  public default void SetAlternatingColors(Color color1, Color color2) {
    SetStripColor((int position) -> {
      return (position % 2 == 0) ? color1 : color2;
    });
  }

  /**
   * Sets the strip to an alternating pair of colors, with even pixels set to
   * color1, and odd pixels set to color2.
   * 
   * @param color1 color to use for even pixels (starting at 0)
   * @param color2 color to use for odd pixels (starting at 1)
   */
  public default void SetAlternatingColors(StockColor color1, StockColor color2) {
    SetAlternatingColors(color1.toWpiColor(), color2.toWpiColor());
  }

  /**
   * Convenience function: sets all controlled lights to a solid color.
   *
   * <p>
   * Note: unlike with various other APIs, the component values for WPI's version
   * of color specification are all as percentages (0.0-1.0).
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
   * Trivial implementation of the interface, for use on robots that don't support
   * lighting (but want to have a subsystem available for convenience/common
   * code).
   */
  public static final class NullLighting extends SubsystemBase implements ILighting {
    /** Constructor. */
    public NullLighting() {
      setName("NullLighting");
    }

    @Override
    public void SetStripColor(ColorSupplier function) {
      // Do nothing with the lighting request.
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
      super.setDefaultCommand(defaultCommand);
    }

    @Override
    public int getLength() {
      return 0;
    }
  }

  /** @return the length of the sequence supported by this object */
  int getLength();

  /**
   * Exposes 'setDefaultCommand' from the WPILib Subsystem class.
   *
   * @param defaultCommand default command to be associated with this subsystem
   */
  void setDefaultCommand(Command defaultCommand);
}
