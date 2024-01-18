// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LightingInterface;

/**
 * An example command that uses a lighting subsystem implementation to establish
 * a single color.
 */
public class SimpleLighting extends Command {
  /** The lighting subsystem we're talking to. */
  private final LightingInterface m_subsystem;

  /** The color to be set for the robot's LEDs. */
  private final LightingInterface.StockColor color;

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleLighting(LightingInterface subsystem) {
    this(subsystem, LightingInterface.StockColor.Green);
  }

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleLighting(LightingInterface subsystem, LightingInterface.StockColor color) {
    setName("Lighting");

    m_subsystem = subsystem;
    this.color = color;
    addRequirements((Subsystem) subsystem);
  }

  @Override
  public void execute() {
    // Do it every time, just in case the lights aren't plugged in @ start.
    m_subsystem.SetStripColor(color);
  }
}
