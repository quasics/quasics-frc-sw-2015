// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class BreathingLights extends CommandBase {
  /** Reference to the Lighting subsystem. */
  final private Lighting m_lighting;
  /** The base color we're using. */
  final private Lighting.StockColor m_color;
  /** Intensity change to be used on each "tick" (50x/sec). */
  final private double m_step;

  /** Intensity used for the last "tick". */
  private double m_currentIntensity = 0;
  /** Iff true, the intensity is increasing on the next "tick". */
  private boolean m_rising = true;

  /**
   * Constructs a command using the specified color and a 1% change on each "tick"
   * (50x/sec).
   * 
   * @param lighting the lighting subsystem
   * @param color    the base color for the lights (adjusted by the intensity)
   * @param step     intensity change to be used on each "tick" (50x/sec)
   */
  public BreathingLights(Lighting lighting, Lighting.StockColor color) {
    this(lighting, color, 0.01);
  }

  /**
   * Constructor.
   * 
   * @param lighting the lighting subsystem
   * @param color    the base color for the lights (adjusted by the intensity)
   * @param step     intensity change to be used on each "tick" (50x/sec)
   */
  public BreathingLights(Lighting lighting, Lighting.StockColor color, double step) {
    this.m_lighting = lighting;
    this.m_color = color;
    this.m_step = step;

    addRequirements(m_lighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start out with lights fully off, and start increasing on the first "tick".
    m_currentIntensity = 0;
    m_rising = true;

    m_lighting.SetStripColor(Lighting.StockColor.Black);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate new intensity (and update direction for next pass, if needed.)
    if (m_rising) {
      m_currentIntensity += m_step;
      if (m_currentIntensity >= 1.0) {
        // Start falling on next round
        m_rising = false;
        m_currentIntensity = 1.0;
      }
    } else {
      m_currentIntensity -= m_step;
      if (m_currentIntensity <= 0) {
        // Start rising on next round
        m_rising = true;
        m_currentIntensity = 0;
      }
    }

    // Update lights, based on current intensity.
    var useColor = m_color.toWpiColor(m_currentIntensity);
    m_lighting.SetStripColor(useColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lighting.SetStripColor(Lighting.StockColor.Black);
  }
}
