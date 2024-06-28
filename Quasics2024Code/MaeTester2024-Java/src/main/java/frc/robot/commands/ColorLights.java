// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

/**
 * Sets the lights on the robot to a specific (fixed) color.
 */
public class ColorLights extends Command {
  final Lights m_lights;
  final int m_r, m_g, m_b;

  /** Creates a new ColorLights, specifying color in individual components. */
  public ColorLights(Lights lights, int r, int g, int b) {
    m_lights = lights;
    m_r = r;
    m_g = g;
    m_b = b;
    addRequirements(lights);
  }

  /** Creates a new ColorLights, specifying color via Color8Bit value. */
  public ColorLights(Lights lights, Color8Bit color) {
    this(lights, color.red, color.green, color.blue);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.setStripColor(m_r, m_g, m_b);
  }
}
