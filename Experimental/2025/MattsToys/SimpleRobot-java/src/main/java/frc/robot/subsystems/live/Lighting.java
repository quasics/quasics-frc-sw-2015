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
import frc.robot.subsystems.interfaces.ICandle;
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

  /**
   * Subset of the lights controlled by this subsystem (may be the full strip).
   */
  private final LightingBuffer m_lightingBuffer;

  /** Subset of the lights reserved to simulate an ICandle. */
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
    this(
        config.lighting().pwmPort(),
        config.lighting().stripLength(),
        (config.hasCandle() && config.candle().simulated()));
  }

  /**
   * Constructor.
   * 
   * TODO: Change this to build a set of views against the underlying buffer,
   * allowing for full logical segmentation of the strip.
   *
   * @param pwmPort             PWM port to which the LED strip is connected
   * @param numLights           number of (logical) lights on the LED strip
   * @param enableCandleSupport whether to enable support for a simulated CANdle
   */
  private Lighting(int pwmPort, int numLights, boolean enableCandleSupport) {
    setName("Lighting");

    System.err.println(
        "Setting up lighting: pwmPort=" + pwmPort +
            ", numLights=" + numLights +
            ", enableCandleSupport=" + enableCandleSupport);

    // Sanity-check inputs.
    if (pwmPort < 0 || pwmPort > 9) {
      throw new IllegalArgumentException("Invalid PWM port: " + pwmPort);
    }

    if (numLights < 0) {
      throw new IllegalArgumentException("Invalid LED strip length: " + numLights);
    } else if (numLights == 0) {
      System.err.println("WARNING: configuring LED strip support with 0 LEDs on it!");
    } else if (enableCandleSupport && numLights <= ICandle.CANDLE_DEFAULT_LENGTH) {
      throw new IllegalArgumentException(
          "Invalid LED strip length for Candle support: " + numLights +
              " (must be at least " + (ICandle.CANDLE_DEFAULT_LENGTH + 1) + ")");
    } else {
      System.err.println("INFO: configuring LED strip support with " + numLights + " LEDs");
    }

    // Configure data members.
    final int reservedCandleLength = enableCandleSupport
        ? ICandle.CANDLE_DEFAULT_LENGTH
        : 0;

    m_ledBuffer = new AddressableLEDBuffer(numLights);

    final AddressableLEDBufferView localView = new AddressableLEDBufferView(
        m_ledBuffer,
        0,
        numLights - (reservedCandleLength + 1));
    m_lightingBuffer = new LightingBuffer(localView);
    m_candleView = enableCandleSupport
        ? new AddressableLEDBufferView(
            m_ledBuffer,
            localView.getLength(),
            numLights - 1)
        : null;

    m_led = new AddressableLED(pwmPort);
    m_led.setLength(m_ledBuffer.getLength());

    initializeStripColors();

    // Start up the LED handling.
    m_led.start();
  }

  private void initializeStripColors() {
    // Start-up lighting
    if (START_CHECKERBOARDED) {
      // On start-up, turn every other pixel on (white).
      SetAlternatingColors(StockColor.White, StockColor.Black);
    } else {
      // On start-up, set to solid (Quasics) green.
      SetStripColor(StockColor.Green.toWpiColor());
    }

    if (m_candleView != null) {
      for (var i = 0; i < m_candleView.getLength(); i++) {
        m_candleView.setLED(i, StockColor.White.toWpiColor());
      }
    }
  }

  public void forceUpdate() {
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  @Override
  public void SetStripColor(ColorSupplier function) {
    m_lightingBuffer.SetStripColor(function);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      SetStripColor(StockColor.Green);
    } else if (DriverStation.isEStopped()) {
      // If we're e-stopped, reflect that....
      SetAlternatingColors(StockColor.Red, StockColor.Blue);
    }

    forceUpdate();
  }

  /**
   * Returns a buffer view for the LED strip to be used in simulating a CANdle
   * device on the robot.
   * 
   * @return the buffer view for use in simulating a CANdle device
   */
  public AddressableLEDBufferView getCandleBuffer() {
    return m_candleView;
  }
}
