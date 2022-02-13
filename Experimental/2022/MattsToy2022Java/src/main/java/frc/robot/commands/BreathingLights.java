// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class BreathingLights extends CommandBase {
  final private Lighting m_lighting;
  final private Lighting.StockColor m_color;
  final private double m_step;

  private double m_currentIntensity = 0;
  private boolean m_rising = true;

  /** Creates a new BreathingLights. */
  public BreathingLights(Lighting lighting, Lighting.StockColor color) {
    this(lighting, color, 0.01);
  }

  /**
   * 
   * @param lighting
   * @param color
   */
  public BreathingLights(Lighting lighting, Lighting.StockColor color, double step) {
    this.m_lighting = lighting;
    this.m_color = color;
    this.m_step = step;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_lighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start out with lights fully off.
    this.m_currentIntensity = 0;
    this.m_rising = true;
    m_lighting.SetStripColor(Lighting.StockColor.Black);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!timer.hasElapsed(0.1)) {
    // return;
    // }

    // Calculate new intensity (and update direction for next pass, if needed.)
    if (this.m_rising) {
      this.m_currentIntensity += this.m_step;
      if (this.m_currentIntensity >= 1.0) {
        // Start falling on next round
        this.m_rising = false;
        this.m_currentIntensity = 1.0;
      }
    } else {
      this.m_currentIntensity -= this.m_step;
      if (this.m_currentIntensity <= 0) {
        // Start rising on next round
        this.m_rising = true;
        this.m_currentIntensity = 0;
      }
    }

    // Update lights, based on current intensity.
    final int r = (int) (m_currentIntensity * m_color.getR());
    final int g = (int) (m_currentIntensity * m_color.getG());
    final int b = (int) (m_currentIntensity * m_color.getB());
    m_lighting.SetStripColor(r, g, b);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lighting.SetStripColor(Lighting.StockColor.Black);
  }
}
