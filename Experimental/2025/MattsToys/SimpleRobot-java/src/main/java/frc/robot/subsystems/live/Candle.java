// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Implementation of the ICandle interface, using a CANdle device.
 */
public class Candle extends SubsystemBase implements ICandle {
  /** Underlying CANdle object being manipulated. */
  private final CANdle m_candle;

  /**
   * Constructor.
   *
   * @param config the configuration for the robot being targeted
   */
  public Candle(RobotConfig config) {
    setName("Candle");

    m_candle = new CANdle(config.candle().canId());
  }

  @Override
  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }
}
