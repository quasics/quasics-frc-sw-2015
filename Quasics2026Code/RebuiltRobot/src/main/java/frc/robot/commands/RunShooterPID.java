// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.real.RealShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooterPID extends Command {
  RealShooter m_shooter;
  private AngularVelocity m_velocityRPM;
  private double m_kickSpeed;

  /** Creates a new RunShooter. */
  public RunShooterPID(RealShooter shooter, AngularVelocity velocity, double kickSpeed) {
    m_shooter = shooter;
    m_velocityRPM = velocity;
    m_kickSpeed = kickSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setFlywheelRPM(m_velocityRPM);
    m_shooter.setKickerSpeed(m_kickSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setFlywheelRPM(m_velocityRPM);
    m_shooter.setKickerSpeed(m_kickSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
}
