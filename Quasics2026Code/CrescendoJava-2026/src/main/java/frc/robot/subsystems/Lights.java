// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Function;

public class Lights extends SubsystemBase {
  // hex codes for commonly used colors
  public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
  public static final Color8Bit WHITE = new Color8Bit(255, 255, 255);
  public static final Color8Bit RED = new Color8Bit(255, 0, 0);
  public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);

  static final int STRIP_LENGTH = 41;

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(STRIP_LENGTH);
  AddressableLED m_led = new AddressableLED(Constants.PwmIDs.LedControl);

  /** Creates a new Lights. */
  public Lights() {
    setSubsystem("Lights");
    m_led.setLength(m_buffer.getLength());
    turnStripOff();
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setStripColor(int red, int green, int blue) {
    setStripColor(new Color8Bit(red, green, blue));
  }

  public void setStripColor(Color8Bit c) {
    setStripColor((Integer i) -> c);
  }

  public void setStripColor(Function<Integer, Color8Bit> colorFcn) {
    for (int i = 0; i < STRIP_LENGTH; i++) {
      m_buffer.setLED(i, colorFcn.apply(i));
    }
    m_led.setData(m_buffer);
  }

  public void turnStripOff() {
    setStripColor(0, 0, 0);
  }

  public int getStripLength() {
    return m_buffer.getLength();
  }
}
