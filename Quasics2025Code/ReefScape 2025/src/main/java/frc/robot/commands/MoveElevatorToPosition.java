// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.AbstractElevator;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {
  AbstractElevator m_elevator;
  double m_targetRotations;

  // TODO: change
  double ALLOWED_POSITION_ERROR = 0;
  double ALLOWED_VELOCITY_ERROR = 0;

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(AbstractElevator elevator, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_targetRotations = rotations;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SparkClosedLoopController pid = m_elevator.getPIDController();
    pid.setReference(m_targetRotations, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_elevator.getVelocity()) > ALLOWED_VELOCITY_ERROR)
      return false;
    if (Math.abs(m_elevator.getPosition() - m_targetRotations) > ALLOWED_POSITION_ERROR)
      return false;
    return true;
  }
}
