// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Simple base class for timed testing.
 */
public class TimedTestBase extends Command {
  final Time m_duration;
  final Timer m_timer = new Timer();

  /**
   * Constructor.
   * 
   * @param duration duration of the test period
   */
  public TimedTestBase(Time duration) {
    m_duration = duration;
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }
}
