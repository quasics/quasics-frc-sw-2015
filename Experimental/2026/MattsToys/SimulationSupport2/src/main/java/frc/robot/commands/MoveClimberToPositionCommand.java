// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IClimber;

/**
 * Command to move the climber to a specified position.
 */
public class MoveClimberToPositionCommand extends Command {
  /** Climber subsystem to control. */
  final IClimber m_climber;

  /** The position to move the climber to. */
  final IClimber.Position m_position;

  /**
   * If true, the command will not finish until the climber has reached the target
   * position; if false, the command will finish immediately after issuing the
   * move command.
   */
  final boolean m_waitUntilFinished;

  /**
   * Creates a new MoveClimberToPositionCommand.
   * 
   * @param climber           the climber subsystem to control.
   * @param position          the position to move the climber to.
   * @param waitUntilFinished if true, the command will not finish until the
   *                          climber has reached the target position; if false,
   *                          the command will finish immediately after issuing
   *                          the move command.
   */
  public MoveClimberToPositionCommand(IClimber climber, IClimber.Position position, boolean waitUntilFinished) {
    m_climber = climber;
    m_position = position;
    m_waitUntilFinished = waitUntilFinished;
    addRequirements(climber.asSubsystem());
  }

  @Override
  public void initialize() {
    m_climber.moveToPosition(m_position);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_climber.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return !m_waitUntilFinished || (m_climber.getCurrentState() != IClimber.State.PidControlled);
  }
}
