// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.ILighting;

/**
 * An example command that uses a lighting subsystem implementation to establish
 * a single color.
 */
public class SimpleLighting extends Command {
  /** The lighting subsystem we're talking to. */
  private final ILighting m_subsystem;

  /** The color to be set for the robot's LEDs. */
  private final ILighting.StockColor color;

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem the subsystem used by this command.
   */
  public SimpleLighting(ILighting subsystem) {
    this(subsystem, ILighting.StockColor.Green);
  }

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem the subsystem used by this command
   * @param color     the color to which the strip should be set
   */
  public SimpleLighting(ILighting subsystem, ILighting.StockColor color) {
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
