// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants;
import java.util.function.Function;

public class Lights extends SubsystemBase {
  public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
  public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  public static final Color8Bit RED = new Color8Bit(255, 0, 0);
  public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
  public static final Color8Bit WHITE = new Color8Bit(255, 255, 255);

  static final int STRIP_LENGTH = 45;

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(STRIP_LENGTH);
  AddressableLED m_led = null; // Only created (in ctor) if hardware is enabled

  private boolean isHardwareDisabled() {
    return ConditionalConstants.SALLY;
  }

  /** Creates a new Lights. */
  public Lights() {
    setSubsystem("Lights");

    if (isHardwareDisabled()) {
      // Simulation only: bail out before we try to use the hardware
      return;
    }

    m_led = new AddressableLED(Constants.PwmIds.LedControl);
    m_led.setLength(m_buffer.getLength());
    turnStripOff();
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the strip to a single, solid color, specified as an RGB value.
   * (Each component must be in the range [0..255].)
   */
  public void setStripColor(int red, int green, int blue) {
    if (isHardwareDisabled()) {
      return;
    }
    setStripColor(new Color8Bit(red, green, blue));
  }

  /**
   * Sets the strip to a single, solid color, specified as an RGB value.
   * (Each component must be in the range [0..255].)
   */
  public void setStripColor(Color8Bit c) {
    if (isHardwareDisabled()) {
      return;
    }
    setStripColor((Integer i) -> c);
  }

  /**
   * Sets the colors for each pixel on the strip, using the specified helper
   * function to get the color at each position.
   */
  public void setStripColor(Function<Integer, Color8Bit> colorFcn) {
    if (isHardwareDisabled()) {
      // Simulation only: bail out before we try to use the hardware
      return;
    }

    for (int i = 0; i < STRIP_LENGTH; i++) {
      m_buffer.setLED(i, colorFcn.apply(i));
    }
    m_led.setData(m_buffer);
  }

  /** Turns all of the pixels on the strip off. */
  public void turnStripOff() {
    if (isHardwareDisabled()) {
      return;
    }
    setStripColor((Integer i) -> BLACK);
  }

  /** Returns the configured length of the LED strip. */
  public int getStripLength() {
    return m_buffer.getLength();
  }
}
