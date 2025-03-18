// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;

public class Candle extends SubsystemBase {
  CANdle m_candle;

  /** Creates a new Candle. */
  public Candle() {
    m_candle = new CANdle(Constants.CanBusIds.CANDLE_CAN_ID);
  }

  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
