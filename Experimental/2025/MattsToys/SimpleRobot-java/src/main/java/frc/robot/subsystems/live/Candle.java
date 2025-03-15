// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting.StockColor;

/**
 * Implementation of the ICandle interface, using a CANdle device.
 */
public class Candle extends SubsystemBase implements ICandle {

  private final CANdle m_candle = new CANdle(Constants.OtherCanIds.CANDLE_ID);

  /** Creates a new Candle. */
  public Candle() {
    setName("Candle");
  }

  @Override
  public void setColor(StockColor color) {
    m_candle.setLEDs(color.r, color.g, color.b);
  }

  @Override
  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }
}
