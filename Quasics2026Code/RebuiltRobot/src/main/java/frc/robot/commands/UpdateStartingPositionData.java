// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IDrivebase;

/**
 * Simple class to update the drivebase's odometry-based positioning at the
 * beginning of a match (i.e., prior to any movement in auto mode).
 * 
 * The intention is to allow the drive team tell the robot where it was
 * actually positioned on the field (e.g., by using a selector on the dashboard
 * that triggers an instance of this command).
 */
public class UpdateStartingPositionData extends Command {
  private final IDrivebase m_drivebase;
  private final Pose2d m_pose;

  /**
   * Constructor.
   * 
   * @param drivebase drivebase being updated
   * @param pose      robot pose, as established pre-match
   */
  public UpdateStartingPositionData(IDrivebase drivebase, Pose2d pose) {
    m_drivebase = drivebase;
    m_pose = pose;

    addRequirements((Subsystem) drivebase);
  }

  @Override
  public void initialize() {
    m_drivebase.updateStartingPosition(m_pose);
  }

  @Override
  public boolean isFinished() {
    // All work is done during initialize
    return true;
  }
}
