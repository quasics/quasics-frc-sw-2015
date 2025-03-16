// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ILighting;

public class LightingBuffer extends SubsystemBase implements ILighting {
  private final AddressableLEDBufferView m_buffer;

  /** Creates a new LightingBuffer. */
  public LightingBuffer(AddressableLEDBufferView view) {
    m_buffer = view;
  }

  public int getLength() {
    return m_buffer.getLength();
  }

  @Override
  public void SetStripColor(ColorSupplier function) {
    for (var i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setLED(i, function.getColorForLed(i));
    }
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }
}
