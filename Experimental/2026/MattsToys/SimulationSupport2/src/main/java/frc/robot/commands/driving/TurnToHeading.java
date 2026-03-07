// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Simple command to turn the robot to a fixed (field-relative) heading.
 */
public class TurnToHeading extends TurnToHeadingBase {
  final Rotation2d m_targetHeading;

  /**
   * Constructor.
   * 
   * @param drivebase     the drivebase being controlled
   * @param targetHeading the heading to which the robot should be turned
   */
  public TurnToHeading(IDrivebasePlus drivebase, Rotation2d targetHeading) {
    super(drivebase);
    m_targetHeading = targetHeading;
  }

  @Override
  protected Rotation2d getTargetHeading() {
    return m_targetHeading;
  }
}
