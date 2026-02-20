// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.real.RealShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooter extends Command {
  RealShooter m_shooter;
  private double m_shooterSpeed;
  private double m_kickerSpeed;
  private boolean m_shooting;

  /** Creates a new RunShooter. */
  public RunShooter(RealShooter shooter, double shooterSpeed, double kickerSpeed, boolean shooting) {
    m_shooter = shooter;
    m_shooterSpeed = shooterSpeed;
    m_kickerSpeed = kickerSpeed;
    m_shooting = shooting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_shooting) {
      m_shooterSpeed = -m_shooterSpeed;
      m_kickerSpeed = -m_kickerSpeed;
    }
    m_shooter.setFlywheelSpeed(m_shooterSpeed);
    m_shooter.setKickerSpeed(m_kickerSpeed);
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
}
