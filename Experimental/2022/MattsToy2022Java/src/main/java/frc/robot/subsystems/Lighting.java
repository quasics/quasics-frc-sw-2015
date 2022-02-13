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

    StockColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }

    public int getR() {
      return r;
    }

    public int getG() {
      return g;
    }

    public int getB() {
      return b;
    }
  }

  final private AddressableLED m_led;
  final private AddressableLEDBuffer m_ledBuffer;

  public Lighting(int pwmPort, int numLights) {
    m_led = new AddressableLED(pwmPort);

    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led.setLength(m_ledBuffer.getLength());

    // On start-up, turn every other pixel on (white).
    final var fixedColor = new edu.wpi.first.wpilibj.util.Color(255, 255, 255);
    for (var i = 0; i < m_ledBuffer.getLength(); i += 2) {
      m_ledBuffer.setLED(i, fixedColor);
    }

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Helper interface to use in populating the data for each LED on the strip. */
  public interface ColorFunctor {
    /**
     * Returns the color to be used for the LED at a given position on the strip.
     */
    public edu.wpi.first.wpilibj.util.Color getColorForLed(int position);
  }

  public void SetStripColor(ColorFunctor function) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, function.getColorForLed(i));
    }

    m_led.setData(m_ledBuffer);
  }

  public void SetStripColor(StockColor color) {
    SetStripColor(color.getR(), color.getG(), color.getB());
  }

  public void SetStripColor(int red, int green, int blue) {
    System.out.println("Setting lights: " + red + "/" + green + "/" + blue);
    final var fixedColor = new edu.wpi.first.wpilibj.util.Color(red / 255.0, green / 255.0, blue / 255.0);
    for (var i = 0; i < m_ledBuffer.getLength(); ++i) {
      m_ledBuffer.setLED(i, fixedColor);
    }

    // Set the data
    m_led.setData(m_ledBuffer);

    // // Defines a "lambda" function that will be used to fulfill the
    // // requirements of the ColorFunctor type. (It will return the
    // // same color for each position in the strip.)
    // final var fixedColor2 = new edu.wpi.first.wpilibj.util.Color(red, green,
    // blue);
    // ColorFunctor function = (var position) -> fixedColor2;
    // // Uses the lambda to set the color for the full strip.
    // SetStripColor(function);
  }
}
