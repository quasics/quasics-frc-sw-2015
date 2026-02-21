// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

public class MoveClimbersAuto extends Command {
  private final Climbers m_climber;
  private final boolean m_extending;

  /** Creates a new MoveClimbersAuto. */
  public MoveClimbersAuto(Climbers climber, boolean extending) {
    m_climber = climber;
    m_extending = extending;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double leftRevolutions = m_climber.GetLeftRevolutions();
    double rightRevolutions = m_climber.GetRightRevolutions();
    if (m_extending) {
      m_climber.ResetRevolutions();
      if (leftRevolutions > -3) {
        // the bool is asking if its the left climber we want
        m_climber.ExtendOneClimber(true);
      }
      if (rightRevolutions > -3) {
        m_climber.ExtendOneClimber(false);
      }
    } else {
      m_climber.SetRevolutions();
      if (leftRevolutions <= 0) {
        // the bool is asking if its the left climber we want
        m_climber.RetractOneClimber(true);
      }
      if (rightRevolutions <= 0) {
        m_climber.RetractOneClimber(false);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftRevolutions = m_climber.GetLeftRevolutions();
    double rightRevolutions = m_climber.GetRightRevolutions();
    if (m_extending) {
      m_climber.ResetRevolutions();
      if (leftRevolutions > -3) {
        // the bool is asking if its the left climber we want
        m_climber.ExtendOneClimber(true);
      }
      if (rightRevolutions > -3) {
        m_climber.ExtendOneClimber(false);
      }
    } else {
      m_climber.SetRevolutions();
      if (leftRevolutions <= 0) {
        // the bool is asking if its the left climber we want
        m_climber.RetractOneClimber(true);
      }
      if (rightRevolutions <= 0) {
        m_climber.RetractOneClimber(false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_extending) {
      if (m_climber.GetLeftRevolutions() < -3
          && m_climber.GetRightRevolutions() < -3) {
        return true;
      }
      return false;
    } else {
      if (m_climber.GetLeftRevolutions() >= 0
          && m_climber.GetRightRevolutions() >= 0) {
        return true;
      }
      return false;
    }
  }
}
