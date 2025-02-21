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
  // CODE_REVIEW: Suggestions for improvement: instead of passing in the number of
  // rotations, specify the target position in terms of an enumeration that is
  // independent of the actual hardware. (See the "TargetPosition" enum I've added
  // to AbstractElevator, and the "getRotationsForPosition()" method I've added to
  // RealElevator, and a version of this command in MoveElevatorToTargetPosiiton
  // that uses this functionality and the assumption of underlying PID control.)
  //
  // This will make it easier to swap out the hardware in the future, change
  // physical values, or to run the same code on different hardware (e.g., in
  // simulation).
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
    // CODE_REVIEW: In theory, you should be able to call this just once (in
    // initialize()), and then that should "stick". The only reason you would need
    // to call this repeatedly in execute() would be if the reference point were
    // changing (e.g., if you're using this to control speed, and that was linked to
    // a driver control or something).

    // TODO: do this
    // SparkClosedLoopController pid = m_elevator.getPIDController();
    // pid.setReference(m_targetRotations, ControlType.kPosition);
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
