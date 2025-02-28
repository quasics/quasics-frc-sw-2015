// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armPivot.ArmPivot;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class MoveArmPivotToPosition extends Command {
  private final ArmPivot m_pivot;
  private final Angle m_angle;

  /** Creates a new MoveArmPivotToPosition. */
  public MoveArmPivotToPosition(ArmPivot pivot, Angle angle) {
    m_pivot = pivot;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setAngleSetpoint(m_angle);
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
