// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class TimedMovementTest extends Command {
  /** Creates a new TimedMovementTest. */
  Timer m_timer = new Timer();
  private final Drivebase m_drivebase;
  private final double m_speed;
  private final Time m_time;

  public TimedMovementTest(Drivebase drivebase, Time time, double speed) {
    m_drivebase = drivebase;
    m_time = time;
    m_speed = speed;
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drivebase.setSpeeds(m_speed, m_speed);
    m_drivebase.enableBreakingMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.setSpeeds(m_speed, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
    m_drivebase.enableBreakingMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time.in(Seconds));
  }
}
