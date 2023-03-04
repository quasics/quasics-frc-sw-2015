// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase implements LightingInterface {
  /** The raw interface to the addressable LED strip connected to the Rio. */
  private final AddressableLED m_led;
  /** The buffer used to set the values for each pixel/LED on the strip. */
  private final AddressableLEDBuffer m_ledBuffer;

  /**
   * Constructor.
   *
   * @param pwmPort PWM port to which the LED strip is connected
   * @param numLights number of (logical) lights on the LED strip
   */
  public Lighting(int pwmPort, int numLights) {
    m_led = new AddressableLED(pwmPort);

    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led.setLength(m_ledBuffer.getLength());

    // On start-up, turn every other pixel on (white).
    final var white = StockColor.White.toWpiColor();
    final var black = StockColor.Black.toWpiColor();
    SetStripColor(
        (int position) -> {
          return (position % 2 == 0) ? white : black;
        });

    // Start up the LED handling.
    m_led.start();
  }

  /**
   * Sets the color for each LED in the strip, using the specified function to generate the values
   * for each position.
   *
   * @param function Function generating the color for each LED
   */
  public void SetStripColor(ColorSupplier function) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, function.getColorForLed(i));
    }

    m_led.setData(m_ledBuffer);
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
