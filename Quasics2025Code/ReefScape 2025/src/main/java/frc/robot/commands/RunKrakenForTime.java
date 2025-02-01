// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRoller;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunKrakenForTime extends Command {
  ArmRoller m_ArmRoller;
  double INTAKE_SPEED = -0.3;
  double EXTAKE_SPEED = 1;
  boolean m_intake;
  Timer m_timer;
  double m_stopTime;

  /** Creates a new RunKrakenForTime. */
  public RunKrakenForTime(ArmRoller armRoller, boolean intake, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmRoller = armRoller;
    m_intake = intake;
    m_stopTime = time;
    m_timer = new Timer();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake) m_ArmRoller.setSpeed(INTAKE_SPEED);
    else m_ArmRoller.setSpeed(EXTAKE_SPEED);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake) m_ArmRoller.setSpeed(INTAKE_SPEED);
    else m_ArmRoller.setSpeed(EXTAKE_SPEED);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmRoller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_stopTime)) return true;
    return false;
  }
}
