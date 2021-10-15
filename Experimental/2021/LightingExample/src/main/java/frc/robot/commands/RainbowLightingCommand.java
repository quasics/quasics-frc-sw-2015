// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class RainbowLightingCommand extends CommandBase {
  /** The subsystem we'll work with to set the lights. */
  final LightingSubsystem subsystem;

  /** Maximum legal value in WPILib for hues. */
  static final int MAX_HUE = 180;

  /**
   * Used to store an "offset" used to translate a physical LED's position to an
   * effective position around the range of hues. This is reset in initialize(),
   * and then updated every time that execute() is called, in order to advance the
   * rainbow a step.
   */
  int offset = 0;

  /**
   * How long (in seconds) the colors will remain set on each LED, before
   * advancing to the next step along the color wheel.
   */
  final double secondsBeforeAdvancing;

  /** Used to control delay in advancing colors. */
  final Timer timer = new Timer();

  private final LightingSubsystem.ColorFunctor colorFunction;

  /** Creates a new RainbowLightingCommand. */
  public RainbowLightingCommand(LightingSubsystem subsystem) {
    this(subsystem, 0, 0);
  }

  /**
   * Creates a new RainbowLightingCommand.
   * 
   * @param subsystem              the lighting subsystem being controlled by the
   *                               command
   * @param secondsBeforeAdvancing how long (in seconds) that the LEDs will remain
   *                               a given color before "advancing" to the next
   *                               stage
   */
  public RainbowLightingCommand(LightingSubsystem subsystem, double secondsBeforeAdvancing) {
    this(subsystem, secondsBeforeAdvancing, 0);
  }

  /**
   * Creates a new RainbowLightingCommand.
   * 
   * @param subsystem              the lighting subsystem being controlled by the
   *                               command
   * @param secondsBeforeAdvancing how long (in seconds) that the LEDs will remain
   *                               a given color before "advancing" to the next
   *                               stage. (Normalized to a minimum of 0.)
   * @param extraGapBetweenColors  any extra "distance" along the color wheel to
   *                               be used between adjacent LEDs. (Normalized to a
   *                               minimum of 0.)
   */
  public RainbowLightingCommand(LightingSubsystem subsystem, double secondsBeforeAdvancing, int extraGapBetweenColors) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.secondsBeforeAdvancing = Math.max(0, secondsBeforeAdvancing);
    final int normalizedExtraGapBetweenColors = Math.max(0, extraGapBetweenColors);

    colorFunction = (var position) -> {
      int effectivePosition = (position + offset + normalizedExtraGapBetweenColors) % MAX_HUE;
      // h - the h value [0-180] - ranges from red @ 0 to green @ 60, to blue @ 120,
      // and back to red
      // s - the s value [0-255] - "depth of color" (lower values shift toward white)
      // (colorfulness, relative to its own brightness)
      // v - the v value [0-255] - brightness
      return Color.fromHSV(effectivePosition, 255, 255);
    };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the values used to control color advance.
    offset = 0;
    timer.reset();
    timer.start();

    // Update the colors of the LEDs on the strip.
    subsystem.SetStripColor(colorFunction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // See if any requested delay has elapsed before updating "offset" to advance
    // the colors along the strip.
    if (secondsBeforeAdvancing == 0 || timer.hasElapsed(secondsBeforeAdvancing)) {
      timer.reset();
      ++offset;
    }

    // Update the colors of the LEDs on the strip.
    subsystem.SetStripColor(colorFunction);
  }
}
