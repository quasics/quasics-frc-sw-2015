// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import java.util.function.Function;

public class RunningLights extends Command {
  private final Time m_stepTime;
  private final Lights m_lights;
  private final int m_pulseSize;
  private final Color8Bit m_color;
  private int m_lastPos;
  Timer m_Timer = new Timer();

  /** Creates a new RunningLights. */
  public RunningLights(Lights lights, Time stepTime, int pulseSize, Color8Bit color) {
    m_stepTime = stepTime;
    m_lights = lights;
    m_pulseSize = pulseSize;
    m_color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_lights == null)
      return;

    m_lights.setStripColor(Lights.BLACK);
    m_lastPos = 0;
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_lights == null) {
      return;
    }
    if (!m_Timer.hasElapsed(m_stepTime.in(Seconds))) {
      return;
    }

    final int pulseStartPosition = (m_lastPos + 1) % m_lights.getStripLength();
    final int pulseEndPosition = (m_lastPos + 1 + m_pulseSize) % m_lights.getStripLength();
    m_lights.setStripColor(position -> {
      if (pulseStartPosition <= pulseEndPosition) {
        // Simple case: turning on lights from [startPos,endPos)
        if (position >= pulseStartPosition && position < (pulseStartPosition + this.m_pulseSize)) {
          return this.m_color;
        }
      } else {
        // Handles wrap-around: turn on lights from [0,endPos] and
        // [startPos,...]
        if (position >= pulseStartPosition || position < pulseEndPosition) {
          return this.m_color;
        }
      }

      // All other lights should be off.
      return Lights.BLACK;
    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.stop();
    if (m_lights != null) {
      m_lights.setStripColor(0, 100, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
