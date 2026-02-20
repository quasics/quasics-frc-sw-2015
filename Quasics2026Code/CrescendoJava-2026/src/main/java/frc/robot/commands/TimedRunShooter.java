// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TimedRunShooter extends Command {
  private final Shooter m_shooter;
  private final double m_shooterSpeed;
  Timer m_timer = new Timer();
  private final Time m_time;

  /** Creates a new TimedRunShooter. */
  public TimedRunShooter(Shooter shooter, double shooterSpeed, Time time, boolean shooting) {
    m_shooter = shooter;
    m_time = time;
    if (shooting) {
      m_shooterSpeed = Math.abs(shooterSpeed);
    } else {
      m_shooterSpeed = -Math.abs(shooterSpeed);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_shooter.setFlywheelSpeed(m_shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setFlywheelSpeed(m_shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time.in(Seconds));
  }
}
