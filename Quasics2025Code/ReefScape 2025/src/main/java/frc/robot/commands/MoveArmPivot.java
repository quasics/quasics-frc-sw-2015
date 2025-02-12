// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPivot;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class MoveArmPivot extends Command {
  private final ArmPivot m_pivot;
  private final double m_pivotSpeed;
  private final boolean m_deploying;
  /** Creates a new MoveArmPivot. */
  public MoveArmPivot(ArmPivot pivot, double pivotSpeed, boolean deploying) {
    m_pivot = pivot;
    m_deploying = deploying;
    if (m_deploying) {
      m_pivotSpeed = Math.abs(pivotSpeed);
    } else {
      m_pivotSpeed = -Math.abs(pivotSpeed);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setArmPivotSpeed(m_pivotSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setArmPivotSpeed(m_pivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
