// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooterForTime extends Command {
  IShooter m_shooter;
  private double m_shooterSpeed;
  private double m_kickerSpeed;
  double m_stopTime;
  Timer m_timer;

  /** Creates a new RunShooterForTime. */
  public RunShooterForTime(
      IShooter shooter, double shooterSpeed, double kickerSpeed, boolean shooting,
      double time) {
    m_shooter = shooter;
    m_shooterSpeed = shooterSpeed;
    m_kickerSpeed = kickerSpeed;
    if (shooting) {
      m_shooterSpeed = Math.abs(shooterSpeed);
      m_kickerSpeed = Math.abs(kickerSpeed);
    } else {
      m_shooterSpeed = -Math.abs(shooterSpeed);
      m_kickerSpeed = -Math.abs(kickerSpeed);
    }
    m_timer = new Timer();
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setFlywheelSpeed(m_shooterSpeed);
    m_shooter.setKickerSpeed(m_kickerSpeed);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setFlywheelSpeed(m_shooterSpeed);
    m_shooter.setKickerSpeed(m_kickerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_stopTime))
      return true;
    return false;
  }
}
