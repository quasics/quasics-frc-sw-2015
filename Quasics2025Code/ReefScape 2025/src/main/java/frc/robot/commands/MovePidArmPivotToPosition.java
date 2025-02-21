// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armPivot.PidFocusedArmPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovePidArmPivotToPosition extends Command {
  private final PidFocusedArmPivot m_pivot;
  private final Angle m_angle;

  /** Creates a new MoveArmPivotToPosition. */
  public MovePidArmPivotToPosition(PidFocusedArmPivot pivot, Angle angle) {
    m_pivot = pivot;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // PIDController pidController = m_pivot.getPivotPIDController();
    // pidController.setSetpoint(m_angle);
    // m_pivot.driveArmToSetpoint(pidController.calculate(m_pivot.getPivotAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pivot.atSetpoint();
  }
}
