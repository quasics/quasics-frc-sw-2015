// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimbersForTime extends Command {

  private final Climbers m_climbers;
  private final boolean m_extending;
  private final double m_speed;
  private final double m_runTime;

  /** Creates a new MoveClimbersForTime. */
  public MoveClimbersForTime(Climbers climbers, boolean extending, double speed, double runTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbers = climbers;
    m_extending = extending;
    m_speed = speed;
    m_runTime = runTime;
    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_extending) {
      m_climbers.startExtending(-m_speed);
    } else {
      m_climbers.startRetracting(m_speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_extending) {
      m_climbers.startExtending(-m_speed);
    } else {
      m_climbers.startRetracting(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Timer time = new Timer();
    time.start();
    if (time.advanceIfElapsed(m_runTime)) {
      return true;
    }
    return false;
  }
}
