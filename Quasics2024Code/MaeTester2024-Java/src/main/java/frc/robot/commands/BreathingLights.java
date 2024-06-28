// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class BreathingLights extends Command {
  final Lights m_lights;
  int m_red = 0, m_green = 0, m_blue = 0;
  boolean m_isIncrementing = true;

  /** Creates a new BreathingLights. */
  public BreathingLights(Lights lights) {
    m_lights = lights;
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.setStripColor(Lights.BLACK);
    m_red = 0;
    m_green = 0;
    m_blue = 0;
    m_isIncrementing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // CODE_REVIEW: This is ignoring red and blue; despite having data members for
  // these, we only ever "breathe green".
  @Override
  public void execute() {
    if (m_green >= 255) {
      m_isIncrementing = false;
    }
    if (m_green <= 0) {
      m_isIncrementing = true;
    }
    if (m_isIncrementing && m_green < 255) {
      m_green = m_green + 1;
    }
    if (!m_isIncrementing && m_green > 0) {
      m_green = m_green - 1;
    }
    m_lights.setStripColor(0, m_green, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lights.setStripColor(Lights.BLACK);
  }
}
