// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractElevator;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class MoveElevatorToPosition extends Command {
  final private AbstractElevator m_elevator;
  final private AbstractElevator.TargetPosition m_target;
  final private boolean m_waitForTargetReached;

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(AbstractElevator elevator, AbstractElevator.TargetPosition target) {
    this(elevator, target, true);
  }

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(AbstractElevator elevator, AbstractElevator.TargetPosition target,
      boolean waitForTargetReached) {
    m_elevator = elevator;
    m_target = target;
    m_waitForTargetReached = waitForTargetReached;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
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
