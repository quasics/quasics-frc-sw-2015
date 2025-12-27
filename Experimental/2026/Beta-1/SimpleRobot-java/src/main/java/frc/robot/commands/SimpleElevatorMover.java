// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IElevator;

/**
 * A simple command that moves an elevator up or down.
 */
public class SimpleElevatorMover extends Command {
  /** Elevator subsystem being controlled. */
  private final IElevator m_elevator;

  /** Direction in which the elevator should be moved. */
  private final Direction m_direction;

  /** Direction of movement. */
  public enum Direction {
    /** Move the elevator up. */
    UP,
    /** Move the elevator down. */
    DOWN
  }

  /**
   * Constructor.
   *
   * @param elevator  the elevator to be moved
   * @param direction the direction in which it should move
   */
  public SimpleElevatorMover(IElevator elevator, Direction direction) {
    m_elevator = elevator;
    m_direction = direction;
    addRequirements(m_elevator.asSubsystem());
  }

  @Override
  public void initialize() {
    if (m_direction == Direction.UP) {
      m_elevator.extend();
    } else {
      m_elevator.retract();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
