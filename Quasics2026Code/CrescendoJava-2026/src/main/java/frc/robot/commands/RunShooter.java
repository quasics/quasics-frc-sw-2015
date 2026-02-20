// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private final Shooter m_shooter;
  private final double m_shooterSpeed;
  // private final boolean m_shooting;

  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double shooterSpeed, boolean shooting) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    if (shooting) {
      m_shooterSpeed = Math.abs(shooterSpeed);
    } else {
      m_shooterSpeed = -Math.abs(shooterSpeed);
    }
    // m_shooting = shooting;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
}
