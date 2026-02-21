// Copyright (c) 2024-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.util.config.RobotConfig;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

/**
 * An example subsystem for controlling an LED strip for custom lighting on the
 * robot.
 *
 * This provides an interface that allows for both configuring the strip as a
 * whole, and allocating "subviews" of the strip for independent control (with
 * the unallocated LEDs at the front of the strip being used as the lights
 * directly supported by this class), which can be assigned to LightingBuffer
 * objects.
 *
 * This class also provides support for simulating a CANdle by injecting the
 * allocation of a subview of the appropriate size during construction, which
 * can then be used with SimCandle.
 *
 * @see frc.robot.subsystems.live.LightingBuffer
 * @see frc.robot.subsystems.simulated.SimCandle
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

  /**
   * Views of subsets of the lights that may be assigned to LightingBuffer
   * objects.
   *
   * @see frc.robot.subsystems.live.LightingBuffer
   */
  List<AddressableLEDBufferView> m_subViews;

  /**
   * Optional ColorSupplier used to control the lighting while we're disabled
   * (e.g., to provide signalling to the drive team while positioning the robot
   * on the field before a match, etc.).
   */
  private ColorSupplier m_disabledColorSupplier = null;

  /**
   * Iff true, the robot will initialize LEDs to alternating black (off) and
   * white; otherwise, they will default to (Quasics) green.
   */
  private static final boolean START_CHECKERBOARDED = false;

  /**
   * Constructor.
   *
   * @param config robot configuration with the lighting subsystem parameters
   */
  public Lighting(RobotConfig config) {
    this(config.lighting().pwmPort(), config.lighting().stripLength(),
        (config.hasCandle() && config.candle().simulated()),
        config.lighting().subViews());
  }

  /**
   * Constructor.
   *
   * @param pwmPort             PWM port to which the LED strip is connected
   * @param numLights           number of (logical) lights on the LED strip
   * @param enableCandleSupport whether to enable support for a simulated CANdle
   * @param subViews            list of lengths for desired subviews
   */
  @SuppressWarnings("unchecked")
  public Lighting(int pwmPort, int numLights, boolean enableCandleSupport,
      List<Integer> subViews) {
    this(pwmPort, numLights,
        Stream
            .concat(Collections.singletonList(ICandle.CANDLE_DEFAULT_LENGTH)
                        .stream(),
                (subViews != null ? subViews : Collections.EMPTY_LIST).stream())
            .toList());
  }

  /**
   * Constructor.
   *
   * @param pwmPort   PWM port to which the LED strip is connected
   * @param numLights number of (logical) lights on the LED strip
   * @param subViews  list of lengths for desired subviews
   */

  public Lighting(int pwmPort, int numLights, List<Integer> subViews) {
    setName("Lighting");

    System.err.println("Setting up lighting: pwmPort=" + pwmPort
        + ", numLights=" + numLights + ", subViews=" + subViews);

    //
    // Sanity-check inputs.
    //
    if (pwmPort < 0 || pwmPort > 9) {
      throw new IllegalArgumentException("Invalid PWM port: " + pwmPort);
    }

    if (numLights < 0) {
      throw new IllegalArgumentException(
          "Invalid LED strip length: " + numLights);
    } else if (numLights == 0) {
      System.err.println(
          "WARNING: configuring LED strip support with 0 LEDs on it!");
    }

    // For simplicity, make sure that we have *some* list of sub-views.
    if (subViews == null) {
      subViews = Collections.emptyList();
    }

    final int subViewsSum = subViews.stream().mapToInt(Integer::intValue).sum();
    if (subViewsSum > 0 && numLights < subViewsSum) {
      throw new IllegalArgumentException(
          "Invalid LED strip length for requested subviews: " + numLights
          + " (must be at least " + subViewsSum + ")");
    } else {
      System.err.println(
          "INFO: configuring LED strip support with " + numLights + " LEDs");
    }
    final int unallocatedLeds = Math.max(numLights - subViewsSum, 0);

    //
    // Configure data members.
    //
    m_ledBuffer = new AddressableLEDBuffer(numLights);
    m_led = new AddressableLED(pwmPort);
    m_led.setLength(m_ledBuffer.getLength());

    // First (local) view is the set of LEDs not allocated to subviews.
    if (unallocatedLeds > 0) {
      m_lightingBuffer = new LightingBuffer(new AddressableLEDBufferView(
          m_ledBuffer, 0, numLights - (subViewsSum + 1)));
    } else {
      m_lightingBuffer = null;
    }

    // Allocate subviews from the remaining LEDs on the strip.
    var viewList = new ArrayList<AddressableLEDBufferView>(subViews.size());
    int runningSum = unallocatedLeds;
    for (int size : subViews) {
      viewList.add(new AddressableLEDBufferView(
          m_ledBuffer, runningSum, runningSum + size - 1));
      runningSum += size;
    }
    m_subViews = Collections.unmodifiableList(viewList);

    //
    // Finish setup
    //

    // Starting colors for each of the strips (selected more or less
    // arbitrarily).
    initializeStripColors();

    // Start up the LED handling.
    m_led.start();
  }

  /**
   * Initializes the different blocks (views) of the strip to various
   * colors/patterns.
   */
  private void initializeStripColors() {
    // Start-up lighting for main (local) view.
    if (START_CHECKERBOARDED) {
      // On start-up, turn every other pixel on (white).
      SetAlternatingColors(StockColor.White, StockColor.Black);
    } else {
      // On start-up, set to solid (Quasics) green.
      SetStripColor(StockColor.Green.toWpiColor());
    }

    // Initialize the other views.
    if (m_subViews != null) {
      int counter = 0;
      for (var view : m_subViews) {
        // Alternate between white and green.
        final Color color = switch (++counter % 3) {
          case 1 -> StockColor.White.toWpiColor();
          case 2 -> StockColor.Purple.toWpiColor();
          default -> StockColor.Orange.toWpiColor();
        };
        for (var i = 0; i < view.getLength(); i++) {
          view.setLED(i, color);
        }
      }
    }
  }

  /**
   * Returns the list of LED buffer views for any configured sub-views.
   *
   * @return the list of LED buffer views
   */
  public List<AddressableLEDBufferView> getSubViews() {
    return m_subViews;
  }

  /** Force the strip to update, reflecting the current buffer contents. */
  public void forceUpdate() {
    m_led.setData(m_ledBuffer);
  }

  /////////////////////////////////////////////////////////////////
  //
  // ILighting methods
  //
  /////////////////////////////////////////////////////////////////

  @Override
  public void SetStripColor(ColorSupplier function) {
    if (m_lightingBuffer == null) {
      return;
    }
    m_lightingBuffer.SetStripColor(function);
  }

  @Override
  public void SetDisabledSupplier(ColorSupplier function) {
    m_disabledColorSupplier = function;
  }

  @Override
  public int getLength() {
    if (m_lightingBuffer == null) {
      return 0;
    }
    return m_lightingBuffer.getLength();
  }

  /////////////////////////////////////////////////////////////////
  //
  // Subsystem methods
  //
  /////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (m_disabledColorSupplier != null) {
        SetStripColor(m_disabledColorSupplier);
      } else {
        SetStripColor(StockColor.Green);
      }
    } else if (DriverStation.isEStopped()) {
      // If we're e-stopped, reflect that....
      SetAlternatingColors(StockColor.Red, StockColor.Blue);
    }

    forceUpdate();
  }

  //
  // Methods from Closeable interface (primarily for unit testing support)
  //

  @Override
  public void close() throws IOException {
    m_led.close();
  }
}
