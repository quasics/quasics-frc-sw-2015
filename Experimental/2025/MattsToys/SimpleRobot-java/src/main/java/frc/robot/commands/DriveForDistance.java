// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

/**
 * Sample command to make the robot move (straight)) for a specific distance, at
 * a given speed.
 */
public class DriveForDistance extends Command {
  /** Drive base being controlled. */
  final IDrivebase m_drivebase;
  /**
   * Desired velocity (as a % of the drive base's maximum), with negative values
   * being "backward".
   */
  final double m_percentSpeed;
  /** Distance to be moved. */
  final Distance m_distance;
  /** Calculated stopping point. */
  Distance m_stopAtPosition;

  /**
   * Creates a new DriveForDistance.
   * 
   * @param drivebase    drive base being controlled
   * @param percentSpeed desired velocity (as a % of the drive base's maximum),
   *                     with negative values
   *                     being "backward"
   * @param distance     distance to be moved
   */
  public DriveForDistance(IDrivebase drivebase, double percentSpeed, Distance distance) {
    m_drivebase = drivebase;
    final double distanceSign = (distance.baseUnitMagnitude() > 0 ? 1 : -1);
    m_percentSpeed = distanceSign * Math.abs(percentSpeed);
    m_distance = distance;
    addRequirements(m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    m_stopAtPosition = m_drivebase.getLeftPosition().plus(m_distance);
    m_drivebase.tankDrive(m_percentSpeed);
  }

  @Override
  public void execute() {
    m_drivebase.tankDrive(m_percentSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    if (m_distance.baseUnitMagnitude() >= 0) {
      // Moving forward
      return m_drivebase.getLeftPosition().gte(m_stopAtPosition);
    } else {
      // Moving backward
      return m_drivebase.getLeftPosition().lte(m_stopAtPosition);
    }
  }
}
