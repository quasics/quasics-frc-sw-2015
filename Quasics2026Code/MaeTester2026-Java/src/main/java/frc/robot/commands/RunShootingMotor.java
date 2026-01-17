// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShootingMotor extends Command {
  private final Shooter m_shooter;
  private final double m_power;
  /** Creates a new RunShootingMotor. */
  public RunShootingMotor(Shooter shooter, double power) {
    m_shooter = shooter;
    m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter power: " + m_power);
    m_shooter.SetSpeed(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
  }
}
