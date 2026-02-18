// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.real.RealShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooter extends Command {
  RealShooter m_shooter;
  double m_velocityRPS;

  /** Creates a new RunShooter. */
  public RunShooter(RealShooter shooter, double velocity, boolean shooting) {
    m_shooter = shooter;
    m_velocityRPS = velocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setFlywheelSpeed(m_velocityRPS);
    // m_shooter.setKickerSpeed(); determine if want one set speed or adjustible
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setFlywheelSpeed(m_velocityRPS);
    // m_shooter.setKickerSpeed(); see above
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
}
