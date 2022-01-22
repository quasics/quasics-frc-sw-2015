// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  /** Creates a new ExampleSubsystem. */
  public Lighting(int pwmPort, int numLights) {
    m_led = new AddressableLED(pwmPort);

    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led.setLength(m_ledBuffer.getLength());

    // On start-up, turn every other pixel on (white).
    final Color fixedColor = new Color(255, 255, 255);
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
    public Color getColorForLed(int position);
  }

  public void SetStripColor(ColorFunctor function) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, function.getColorForLed(i));
    }

    m_led.setData(m_ledBuffer);
  }

  public void SetStripColor(int red, int green, int blue) {
    // Defines a "lambda" function that will be used to fulfill the
    // requirements of the ColorFunctor type. (It will return the
    // same color for each position in the strip.)
    final Color fixedColor = new Color(red, green, blue);
    ColorFunctor function = (var position) -> fixedColor;

    // Uses the lambda to set the color for the full strip.
    SetStripColor(function);
  }
}
