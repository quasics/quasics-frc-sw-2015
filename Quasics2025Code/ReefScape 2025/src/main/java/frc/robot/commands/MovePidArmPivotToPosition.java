// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armPivot.PidFocusedArmPivot;

/**
 * Arm movement command based on the underlying subsystem handling PID directly.
 */
public class MovePidArmPivotToPosition extends Command {
  private final PidFocusedArmPivot m_pivot;
  private final Angle m_angle;

  /** Constructor. */
  public MovePidArmPivotToPosition(PidFocusedArmPivot pivot, Angle angle) {
    m_pivot = pivot;
    m_angle = angle;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
    m_pivot.setAngleSetpoint(m_angle);
  }

  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return m_pivot.atSetpoint();
  }
}
