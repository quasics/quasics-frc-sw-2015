// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.subsystems.AbstractElevator.Mode;

/**
 * Simple command to raise the elevator to its highest position. This relies
 * upon the "safety mode" functionality in the elevator, which assumes that it
 * will automatically stop when it reaches maximum extension.
 */
public class MoveElevatorToExtreme extends Command {
  final AbstractElevator m_elevator;
  final boolean m_fullyExtend;

  /** Convenience class. */
  public static final class RetractElevator extends MoveElevatorToExtreme {
    public RetractElevator(AbstractElevator elevator) {
      super(elevator, false);
    }
  }

  /** Convenience class. */
  public static final class RaiseElevator extends MoveElevatorToExtreme {
    public RaiseElevator(AbstractElevator elevator) {
      super(elevator, true);
    }
  }

  /** Creates a new RaiseElevator. */
  public MoveElevatorToExtreme(AbstractElevator elevator, boolean fullyExtend) {
    m_elevator = elevator;
    m_fullyExtend = fullyExtend;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    if (m_fullyExtend && !m_elevator.extend()) {
      System.err.println("*** Warning: couldn't start extending the elevator!");
    } else if (!m_fullyExtend && !m_elevator.retract()) {
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
