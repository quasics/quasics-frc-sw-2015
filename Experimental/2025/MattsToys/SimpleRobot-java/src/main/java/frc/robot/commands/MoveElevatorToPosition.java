// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {
  final private AbstractElevator m_elevator;
  final private AbstractElevator.TargetPosition m_target;

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(AbstractElevator elevator, AbstractElevator.TargetPosition target) {
    m_elevator = elevator;
    m_target = target;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setTargetPosition(m_target);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
