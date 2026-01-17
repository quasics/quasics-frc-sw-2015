// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ILighting;
import java.io.IOException;

/**
 * Implements a subsystem that is wrapped around an AddressableLEDBufferView,
 * allowing for independent control of different segments of a lighting strip.
 */
public class LightingBuffer extends SubsystemBase implements ILighting {
  /** The buffer view being manipulated by this subsystem. */
  private final AddressableLEDBufferView m_buffer;

  private ColorSupplier m_disabledColorSupplier = null;

  /**
   * Creates a new LightingBuffer.
   *
   * @param view the buffer view to be manipulated by this subsystem
   */
  public LightingBuffer(AddressableLEDBufferView view) {
    m_buffer = view;
  }

  @Override
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
  public void SetDisabledSupplier(ColorSupplier function) {
    m_disabledColorSupplier = function;
  }

  @Override
  public void periodic() {
    if (m_disabledColorSupplier != null && DriverStation.isDisabled()) {
      SetStripColor(m_disabledColorSupplier);
    }
  }

  @Override
  public void close() throws IOException {
    // No-op: we have no resources to be closed.
  }
}
