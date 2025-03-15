// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.AbstractElevator;

/**
 * Tells the elevator (to use PID-based control) to move to a specified height.
 *
 * Note that this is a "set it and done" sort of command, since *all* the
 * command needs to do is to configure the target position for the elevator.
 * Once that's been done, the PID logic on the elevator (or a simulated version
 * of it, at least) should continue driving the elevator subsystem to the
 * specified position.
 *
 * Note also, however, that if there's some other command configured as the
 * default on the elevator (e.g., something that reads a joystick control for
 * height, etc.), then that could conflict with this approach.
 */
public class ElevatorToPositionOnController extends Command {
  final private AbstractElevator m_elevator;
  final private AbstractElevator.TargetPosition m_targetPosition;

  /** Creates a new ElevatorToPositionOnController. */
  public ElevatorToPositionOnController(
      AbstractElevator elevator, AbstractElevator.TargetPosition targetPosition) {
    m_elevator = elevator;
    m_targetPosition = targetPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setTargetPosition(m_targetPosition);
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
