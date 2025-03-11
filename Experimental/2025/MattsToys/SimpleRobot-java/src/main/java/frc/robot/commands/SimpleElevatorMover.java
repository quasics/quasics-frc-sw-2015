// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.abstracts.AbstractElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleElevatorMover extends Command {
  private final AbstractElevator m_elevator;
  private final Direction m_direction;

  public enum Direction {
    UP, DOWN
  }

  /** Creates a new SimpleElevatorMover. */
  public SimpleElevatorMover(AbstractElevator elevator, Direction direction) {
    m_elevator = elevator;
    m_direction = direction;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_direction == Direction.UP) {
      m_elevator.extend();
    } else {
      m_elevator.retract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
