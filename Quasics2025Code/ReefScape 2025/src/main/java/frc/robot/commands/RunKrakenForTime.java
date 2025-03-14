// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRoller;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RunKrakenForTime extends Command {
  ArmRoller m_armRoller;
  double m_speed;
  double m_stopTime;
  Timer m_timer;

  /** Creates a new RunKrakenForTime. */
  public RunKrakenForTime(ArmRoller armRoller, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armRoller = armRoller;
    m_speed = speed;
    m_stopTime = time;
    m_timer = new Timer();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armRoller.setSpeed(m_speed);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armRoller.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armRoller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_stopTime))
      return true;
    return false;
  }
}
