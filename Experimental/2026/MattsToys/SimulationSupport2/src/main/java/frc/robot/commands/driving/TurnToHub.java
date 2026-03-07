// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.util.games.rebuilt.TargetPositioningUtils;

/**
 * Example command to turn the robot to point at our alliance's hub (e.g., to
 * line up our shots).
 */
public class TurnToHub extends TurnToHeadingBase {

  /**
   * Constructor.
   * 
   * @param drivebase drivebase being controlled
   */
  public TurnToHub(IDrivebasePlus drivebase) {
    super(drivebase);
  }

  @Override
  protected Rotation2d getTargetHeading() {
    return TargetPositioningUtils.getAngleToHubCenter(m_drivebase.getEstimatedPose());
  }
}
