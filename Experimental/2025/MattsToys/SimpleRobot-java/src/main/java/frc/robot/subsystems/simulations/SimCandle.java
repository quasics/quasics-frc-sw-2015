// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ICandle;

/**
 * Simulated implementation of the ICandle interface (using an
 * AddressableLEDBufferView from the Lighting subsystem).
 */
public class SimCandle extends SubsystemBase implements ICandle {
  /** The LED buffer view used to simulate the CANdle's functionality. */
  final AddressableLEDBufferView m_candleView;

  /**
   * Creates a new SimCandle.
   * 
   * @param view the LED buffer view to use for the simulation
   */
  public SimCandle(AddressableLEDBufferView view) {
    // The CANdle is assumed to be simulated as the 1st subview.
    m_candleView = view;
  }

  @Override
  public void setColor(int r, int g, int b) {
    for (var i = 0; i < m_candleView.getLength(); i++) {
      m_candleView.setRGB(i, r, g, b);
    }
  }
}
