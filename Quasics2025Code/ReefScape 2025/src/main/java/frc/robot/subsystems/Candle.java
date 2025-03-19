// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {
  static public final Color8Bit ORANGE = new Color8Bit(255, 165, 0);
  static public final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  static public final Color8Bit RED = new Color8Bit(255, 0, 0);
  static public final Color8Bit BLACK = new Color8Bit(0, 0, 0);

  CANdle m_candle;

  /** Creates a new Candle. */
  public Candle() {
    m_candle = new CANdle(Constants.CanBusIds.CANDLE_CAN_ID);

    CANdleConfiguration configAll = new CANdleConfiguration();

    // LEDs built into the device are RGB
    configAll.stripType = LEDStripType.RGB;

    // Turn off Status LED when CANdle is actively being controlled
    configAll.statusLedOffWhenActive = true;

    // Leave LEDs on when Loss of Signal occurs
    configAll.disableWhenLOS = false;

    // Dim the LEDs to 50% brightness
    configAll.brightnessScalar = 0.5;

    // True to turn off the 5V rail. This turns off the on-board LEDs as well.
    // (So if we're using the on-board LEDs, it should be false.)
    configAll.v5Enabled = false;

    m_candle.configAllSettings(configAll, 100);
  }

  public void setColor(Color8Bit color) {
    setColor(color.red, color.green, color.blue);
  }

  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }

  @Override
  public void periodic() {
    if (DriverStation.isEStopped()) {
      // Orange when e-stopped.
      setColor(ORANGE);
    }
    // This method will be called once per scheduler run
  }
}
