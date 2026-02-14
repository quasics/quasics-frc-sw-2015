// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example of how you can control an addressable LED strip on an FRC robot.
 */
public class Lighting extends SubsystemBase {
  /** The raw interface to the addressable LED strip connected to the Rio. */
  private final AddressableLED m_led;

  /** The buffer used to set the values for each pixel/LED on the strip. */
  private final AddressableLEDBuffer m_ledBuffer;

  /**
   * Constructor. Creates the subsystem and sets the lights to green as an initial
   * color.
   * 
   * @param pwmPort   PWM port to which the LED strip has been connected
   * @param numLights number of lights (pixels) on the LED strip
   */
  public Lighting(int pwmPort, int numLights) {
    // Let the other version of our constructor do the work: we'll just need to
    // specify the initial color to be used.
    this(
        pwmPort, numLights,
        // Start with the lights set to "Quasics green"
        new Color8Bit(0, 255, 0));
  }

  /**
   * Constructor. Creates the subsystem and sets the lights to the specified
   * initial color.
   * 
   * @param pwmPort      PWM port to which the LED strip has been connected
   * @param numLights    number of lights (pixels) on the LED strip
   * @param initialColor the initial color to which the strip should be set
   */
  public Lighting(int pwmPort, int numLights, Color8Bit initialColor) {
    setName("Lighting");

    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led = new AddressableLED(pwmPort);
    m_led.setLength(m_ledBuffer.getLength());

    // Start up the LED handling.
    m_led.start();

    // Set the initial color (green) for the lights.
    setStripColor(initialColor);
  }

  public void setStripColor(Color8Bit color) {
    // Set the color value for each pixel on the strip.
    for (int i = 0; i < m_ledBuffer.getLength(); ++i) {
      m_ledBuffer.setLED(i, color);
    }

    // Tell the strip to update itself, using the data in the buffer.
    m_led.setData(m_ledBuffer);
  }
}
