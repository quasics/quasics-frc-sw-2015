// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.AbstractElementVisitor14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.AbstractElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToTargetPosition extends Command {
  final private AbstractElevator m_elevator;
  final private AbstractElevator.TargetPosition m_targetPosition;

  /** Creates a new MoveElevatorToTargetPosition. */
  public MoveElevatorToTargetPosition(AbstractElevator elevator, AbstractElevator.TargetPosition targetPosition) {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getVelocity() == 0;
  }
}
