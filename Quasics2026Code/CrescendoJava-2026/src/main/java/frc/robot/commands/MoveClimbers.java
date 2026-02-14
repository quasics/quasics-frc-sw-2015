// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

public class MoveClimbers extends Command {
  private final Climbers m_climber;
  private final boolean m_extending;

  /** Creates a new MoveClimbers. */
  public MoveClimbers(Climbers climber, boolean extending) {
    m_extending = extending;
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_extending) {
      m_climber.StartExtending();
    } else {
      m_climber.StartRetracting();
    }

    m_climber.EnableBraking(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_extending) {
      m_climber.StartExtending();
    } else {
      m_climber.StartRetracting();
    }

    m_climber.EnableBraking(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
    m_climber.EnableBraking(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_extending) {
      if (m_climber.IsFullyExtended()) {
        return true;
      }
      return false;
    } else {
      if (m_climber.IsFullyRetracted()) {
        return true;
      }
      return false;
    }
  }
}
