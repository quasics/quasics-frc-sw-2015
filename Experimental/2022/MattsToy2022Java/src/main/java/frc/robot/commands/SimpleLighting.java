// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Lighting;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses the lighting subsystem. */
public class SimpleLighting extends CommandBase {
  private final Lighting m_subsystem;
  private final Lighting.Color color;

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleLighting(Lighting subsystem) {
    this(subsystem, Lighting.Color.Green);
  }

  /**
   * Creates a new SimpleLighting.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleLighting(Lighting subsystem, Lighting.Color color) {
    setName("Lighting");

    m_subsystem = subsystem;
    this.color = color;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // Do it every time, just in case the lights aren't plugged in @ start.
    m_subsystem.SetStripColor(color);
  }
}
