// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IElevator;

/**
 * Moves the elevator to a specified target height (using PID control), and
 * optionally waiting to signal completion until the elevator gets to that
 * point.
 */
public class MoveElevatorToPosition extends Command {
  /** Elevator being controlled. */
  final private IElevator m_elevator;
  /** Target position. */
  final private IElevator.TargetPosition m_target;
  /**
   * Iff true, command will wait to signal "finished" until the elevator has
   * reached the target.
   */
  final private boolean m_waitForTargetReached;

  /**
   * Contructor. Will wait until the elevator is in position before signalling
   * "finished".
   *
   * @param elevator elevator being controlled
   * @param target   target position
   */
  public MoveElevatorToPosition(IElevator elevator, IElevator.TargetPosition target) {
    this(elevator, target, true);
  }

  /**
   * Contructor.
   *
   * @param elevator             elevator being controlled
   * @param target               target position
   * @param waitForTargetReached indicates if the command should wait to signal
   *                             "finished" until the elevator reaches the target
   *                             position
   */
  public MoveElevatorToPosition(
      IElevator elevator,
      IElevator.TargetPosition target,
      boolean waitForTargetReached) {
    m_elevator = elevator;
    m_target = target;
    m_waitForTargetReached = waitForTargetReached;

    addRequirements(elevator.asSubsystem());
  }

  @Override
  public void initialize() {
    m_elevator.setTargetPosition(m_target);
  }

  @Override
  public void end(boolean interrupted) {
    if (m_waitForTargetReached || interrupted) {
      m_elevator.stop();
    }
  }

  /**
   * @return true when the command should end (either because we're not waiting
   *         for the target position to be reached, or because we *have* reached
   *         it)
   */
  @Override
  public boolean isFinished() {
    if (m_waitForTargetReached) {
      // Waiting until the elevator gets to the target height.
      return m_elevator.atTargetPosition();
    } else {
      // OK: we're just going to let the PID control in the elevator subsystem get us
      // there.
      return true;
    }
  }
}
