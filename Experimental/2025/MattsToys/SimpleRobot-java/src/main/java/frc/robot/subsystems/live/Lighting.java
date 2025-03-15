// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * An example subsystem for controlling an LED strip for custom lighting on the
 * robot.
 */
public class Lighting extends SubsystemBase implements ILighting {
  /** The raw interface to the addressable LED strip connected to the Rio. */
  private final AddressableLED m_led;

  /** The buffer used to set the values for each pixel/LED on the strip. */
  private final AddressableLEDBuffer m_ledBuffer;

  private static final int CANDLE_LENGTH = 8;

  private final AddressableLEDBufferView m_lightingView;
  private final AddressableLEDBufferView m_candleView;

  /**
   * Iff true, the robot will initialize LEDs to alternating black (off) and
   * white; otherwise, they will default to (Quasics) green.
   */
  private static final boolean START_CHECKERBOARDED = false;

  /**
   * Constructs a lighting subsystem for the specified robot.
   *
   * @param config the configuration for the robot being targeted
   */
  public Lighting(RobotConfig config) {
    this(config.lighting().pwmPort(), config.lighting().stripLength(), config.hasCandle());
  }

  /**
   * Constructor.
   *
   * @param pwmPort   PWM port to which the LED strip is connected
   * @param numLights number of (logical) lights on the LED strip
   */
  private Lighting(int pwmPort, int numLights, boolean enableCandleSupport) {
    setName("Lighting");

    System.err.println("Setting up lighting: port=" + pwmPort + ", length=" + numLights);

    // Sanity-check inputs.
    if (pwmPort < 0 || pwmPort > 9) {
      throw new IllegalArgumentException("Invalid PWM port: " + pwmPort);
    }

    if (numLights < 0) {
      throw new IllegalArgumentException("Invalid LED strip length: " + numLights);
    } else if (numLights == 0) {
      System.err.println("WARNING: configuring LED strip support with 0 LEDs on it!");
    } else {
      System.err.println("INFO: configuring LED strip support with " + numLights + " LEDs");
    }

    // Configure data members.
    m_led = new AddressableLED(pwmPort);
    final int reservedCandleLength = enableCandleSupport ? CANDLE_LENGTH : 0;
    m_ledBuffer = new AddressableLEDBuffer(numLights + reservedCandleLength);
    m_lightingView = new AddressableLEDBufferView(m_ledBuffer, 0, numLights - 1);
    m_candleView = enableCandleSupport
        ? new AddressableLEDBufferView(m_ledBuffer, numLights, numLights + reservedCandleLength - 1)
        : null;
    m_led.setLength(m_ledBuffer.getLength());

    // Start-up lighting
    if (START_CHECKERBOARDED) {
      // On start-up, turn every other pixel on (white).
      SetAlternatingColors(StockColor.White, StockColor.Black);
    } else {
      // On start-up, set to solid (Quasics) green.
      SetStripColor(StockColor.Green.toWpiColor());
    }

    if (enableCandleSupport) {
      for (var i = 0; i < CANDLE_LENGTH; i++) {
        m_candleView.setLED(i, StockColor.White.toWpiColor());
      }
    }

    // Start up the LED handling.
    m_led.start();
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  /**
   * Sets the color for each LED in the strip, using the specified function to
   * generate the values
   * for each position.
   *
   * @param function Function generating the color for each LED
   */
  public void SetStripColor(ColorSupplier function) {
    for (var i = 0; i < m_lightingView.getLength(); i++) {
      m_lightingView.setLED(i, function.getColorForLed(i));
    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      SetStripColor(StockColor.Green);
    } else if (DriverStation.isEStopped()) {
      // If we're e-stopped, reflect that....
      SetAlternatingColors(StockColor.Red, StockColor.Blue);
    }
  }

  /**
   * @return the buffer view for use in simulating a CANdle device
   */
  AddressableLEDBufferView getCandleBuffer() {
    return m_candleView;
  }
}
