// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.util.RobotConfigs.RobotConfig;

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

    CANdleConfiguration configAll = new CANdleConfiguration();

    // LEDs built into the device are RGB
    configAll.LED.StripType = StripTypeValue.RGB;

    // Turn off Status LED when CANdle is actively being controlled
    configAll.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    
    // Leave LEDs on when Loss of Signal occurs
    configAll.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

    // Dim the LEDs to 50% brightness
    configAll.LED.BrightnessScalar = 0.5;

    // True to turn off the 5V rail. This turns off the on-board LEDs as well.
    // (So if we're using the on-board LEDs, it should be false.)
    configAll.CANdleFeatures.Enable5VRail = Enable5VRailValue.Disabled;

    m_candle.getConfigurator().apply(configAll);
  }

  /**
   * Set the state of the LEDs based on the overall state of the robot.
   *
   * @see #periodic()
   */
  protected void updateLedsLocally() {
    StockColor color = StockColor.Green;
    double intensity = 1.0;

    if (!DriverStation.isEnabled()) {
      intensity = 0.2;
    }

    var optAlliance = DriverStation.getAlliance();
    if (optAlliance.isEmpty()) {
      color = StockColor.Green;
    } else {
      var alliance = optAlliance.get();
      color = switch (alliance) {
        case Blue -> StockColor.Blue;
        case Red -> StockColor.Red;
      };
    }

    // Override the color, based on other conditions
    if (DriverStation.isEStopped()) {
      color = StockColor.Orange;
    }

    setIntensity(intensity);
    setColor(color);
  }

  //////////////////////////////////////////////////////////////////////
  //
  // ICandle functions
  //
  //////////////////////////////////////////////////////////////////////

  @Override
  public void setIntensity(double intensity) {
    m_candle.configBrightnessScalar(intensity);
  }

  @Override
  public void setColor(int r, int g, int b) {
    m_candle.setLEDs(r, g, b);
  }

  //////////////////////////////////////////////////////////////////////
  //
  // SubsystemBase functions
  //
  //////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    super.periodic();

    if (super.getCurrentCommand() == null) {
      // No command is using us right now, so we'll do things for ourselves.
      ICandle.updateCandleForAllianceAndStatus(this);
    }
  }
}
