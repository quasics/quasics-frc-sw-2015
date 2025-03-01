// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.armPivot.AbstractArmPivot;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class MoveArmPivot extends Command {
  private final AbstractArmPivot m_pivot;
  private final double m_pivotSpeed;

  /** Creates a new MoveArmPivot. */
  public MoveArmPivot(AbstractArmPivot pivot, double pivotSpeed) {
    m_pivot = pivot;
    m_pivotSpeed = pivotSpeed;
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
    /*
     * Angle currentAngle = m_pivot.getPivotAngle();
     * if (m_direction == Direction.UP) {
     * if (currentAngle.gte(Constants.DesiredEncoderValues.ARM_UP_IN_RADIANS)) {
     * return true;
     * }
     * } else {
     * if (m_direction == Direction.DOWN) {
     * if (currentAngle.lte(Constants.DesiredEncoderValues.ARM_DOWN_IN_RADIANS)) {
     * return true;
     * }
     * }
     * }
     * 
     * /*
     * double position = m_pivot.getRawPivotPosition();
     * if (m_direction == Direction.UP) {
     * if (position > Constants.DesiredEncoderValues.ARM_UP && position < 0.95) {
     * return true;
     * }
     * } else {
     * if (position < Constants.DesiredEncoderValues.ARM_DOWN) {
     * return true;
     * }
     * }
     */
    return false;
  }
}
