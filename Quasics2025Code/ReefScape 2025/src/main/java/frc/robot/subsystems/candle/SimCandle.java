// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.candle;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Implements the AbstractCandle base under simulation.  (Uses AddressableLED support to show up.)
 */
public class SimCandle extends AbstractCandle {
  double intensity = 1;
  Color8Bit color = new Color8Bit(255, 255, 255);
  final AddressableLED led = new AddressableLED(7);
  final AddressableLEDBuffer buffer = new AddressableLEDBuffer(8);

  /** Creates a new SimCandle. */
  public SimCandle() {
    led.setLength(buffer.getLength());
    led.start();
    updateLeds();
  }

  /** Updates the LEDs (e.g., when color/intensity changes). */
  private void updateLeds() {
    Color8Bit useColor = new Color8Bit((int) (intensity * color.red),
        (int) (intensity * color.green), (int) (intensity * color.blue));

    for (int i = 0; i < buffer.getLength(); ++i) {
      buffer.setLED(i, useColor);
    }
    led.setData(buffer);
  }

  @Override
  public void setIntensity(double value) {
    this.intensity = value;
    updateLeds();
  }

  @Override
  public void setColor(Color8Bit color) {
    this.color = color;
    updateLeds();
  }
}
