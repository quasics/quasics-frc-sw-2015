// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimberForTime extends Command {
  IClimber m_climber;
  private double m_climberSpeed;

  double m_stopTime;
  Timer m_timer;

  /** Creates a new ClimbForTime. */
  public RunClimberForTime(
      IClimber climber, double climbSpeed,
      double time) {
    m_climber = climber;
    m_climberSpeed = climbSpeed;
    m_timer = new Timer();
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setClimberSpeed(m_climberSpeed);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberSpeed(m_climberSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_stopTime))
      return true;
    return false;
  }
}
