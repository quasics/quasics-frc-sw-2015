// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.live.Lighting;

/**
 * Simulated implementation of the ICandle interface (using an
 * AddressableLEDBufferView from the Lighting subsystem).
 */
public class SimCandle extends SubsystemBase implements ICandle {
  final AddressableLEDBufferView m_candleView;

  /** Creates a new SimCandle. */
  public SimCandle(Lighting lighting) {
    m_candleView = lighting.getCandleBuffer();
  }

  @Override
  public void setColor(int r, int g, int b) {
    for (var i = 0; i < m_candleView.getLength(); i++) {
      m_candleView.setRGB(i, r, g, b);
    }
  }
}
