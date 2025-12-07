// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.Mode;

/**
 * Simple command to raise the elevator to its highest position (manually/not
 * using PID). This relies upon the "safety mode" functionality in the elevator,
 * which assumes that it will automatically stop when it reaches maximum
 * extension.
 */
public class MoveElevatorToExtreme extends Command {
  /** Elevator being controlled. */
  final IElevator m_elevator;
  /** Indicates if the elevator should be raised (true) or lowered (false). */
  final boolean m_fullyRaise;

  /** Utility class that only lowers the elevator. */
  public static final class LowerElevator extends MoveElevatorToExtreme {
    /**
     * Constructor.
     *
     * @param elevator elevator being controlled
     */
    public LowerElevator(IElevator elevator) {
      super(elevator, false);
    }
  }

  /** Utility class that only raises the elevator. */
  public static final class RaiseElevator extends MoveElevatorToExtreme {
    /**
     * Constructor.
     *
     * @param elevator elevator being controlled
     */
    public RaiseElevator(IElevator elevator) {
      super(elevator, true);
    }
  }

  /**
   * Constructor.
   *
   * @param elevator      elevator being controlled
   * @param raiseElevator if the elevator should be raised (or lowered, if false)
   */
  public MoveElevatorToExtreme(IElevator elevator, boolean raiseElevator) {
    m_elevator = elevator;
    m_fullyRaise = raiseElevator;
    addRequirements(m_elevator.asSubsystem());
  }

  @Override
  public void initialize() {
    if (m_fullyRaise && !m_elevator.extend()) {
      System.err.println("*** Warning: couldn't start extending the elevator!");
    } else if (!m_fullyRaise && !m_elevator.retract()) {
      System.err.println("*** Warning: couldn't start retracting the elevator!");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.getMode() == Mode.Stopped;
  }
}
