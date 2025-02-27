// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armPivot.AbstractArmPivot;
import frc.robot.subsystems.elevator.AbstractElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmPivotAndElevatorToPosition extends Command {
  private final AbstractArmPivot m_armPivot;
  private final AbstractElevator m_elevator;
  final private Angle m_armPivotSetpoint;
  final private AbstractElevator.TargetPosition m_targetPosition;

  /** Creates a new MoveArmPivotAndElevatorToPosition. */
  public MoveArmPivotAndElevatorToPosition(AbstractArmPivot armPivot, AbstractElevator elevator, Angle armPivotSetpoint,
      AbstractElevator.TargetPosition targetPosition) {
    m_armPivot = armPivot;
    m_elevator = elevator;
    m_armPivotSetpoint = armPivotSetpoint;
    m_targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
    addRequirements(m_armPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armPivot.setAngleSetpoint(m_armPivotSetpoint);
    m_elevator.setTargetPosition(m_targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armPivot.stop();
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(m_armPivot.atSetpoint() && )
    return false;
  }
}
