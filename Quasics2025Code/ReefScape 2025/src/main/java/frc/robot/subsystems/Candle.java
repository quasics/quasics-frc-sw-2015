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

/**
 * "Live" implementation of the ICandle interface.
 */
public class Candle extends SubsystemBase implements ICandle {
  static public final Color8Bit ORANGE = new Color8Bit(255, 165, 0);
  static public final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  static public final Color8Bit BLUE = new Color8Bit(0, 0, 255);
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

  public void setIntensity(double intensity) {
    m_candle.configBrightnessScalar(intensity);
  }

  public void setColor(Color8Bit color) {
    setColor(color.red, color.green, color.blue);
  }

  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }

  /**
   * Set the state of the LEDs based on the overall state of the robot.
   *
   * @see #periodic()
   */
  private void updateLedsLocally() {
    Color8Bit color = Candle.GREEN;
    double intensity = 1.0;

    if (!DriverStation.isEnabled()) {
      intensity = 0.2;
    }

    var optAlliance = DriverStation.getAlliance();
    if (optAlliance.isEmpty()) {
      color = Candle.GREEN;
    } else {
      var alliance = optAlliance.get();
      color = switch (alliance) {
        case Blue -> Candle.BLUE;
        case Red -> Candle.RED;
      };
    }

    // Override the color, based on other conditions
    if (DriverStation.isEStopped()) {
      color = ORANGE;
    }

    setIntensity(intensity);
    setColor(color);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (super.getCurrentCommand() == null) {
      // No command is using us right now, so we'll do things for ourselves.
      updateLedsLocally();
    }
  }
}
