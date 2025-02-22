// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveForDistance extends Command {
  final static double EPSILON = 0.01;

  final IDrivebase m_drivebase;
  final double m_percentSpeed;
  final Distance m_distance;
  Distance m_stopAtPosition;

  /** Creates a new DriveForDistance. */
  public DriveForDistance(IDrivebase drivebase, double percentSpeed, Distance distance) {
    m_drivebase = drivebase;
    m_percentSpeed = (distance.baseUnitMagnitude() > 0 ? 1 : -1) * Math.abs(percentSpeed);
    m_distance = distance;
    addRequirements(m_drivebase.asSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stopAtPosition = m_drivebase.getLeftPosition().plus(m_distance);
    // TODO: Add deadband handling (i.e., if distance is basically 0).
    m_drivebase.setSpeed(m_percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.setSpeed(m_percentSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
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
