// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveForDistance extends Command {
  /** The subsystem we're using to drive. */
  final AbstractDrivebase m_drivebase;

  /** How fast we should be moving. */
  final double m_percentSpeed;

  /** How far we should travel when the command is triggered. */
  final Distance m_distance;

  /**
   * The position at which we should stop driving. (Reset every time that the
   * command is initialized.)
   */
  Distance m_stopAtPosition;

  /**
   * Creates a new DriveForDistance, using our *other* constructor to do the work.
   *
   * @param drivebase    The drivebase to use.
   * @param percentSpeed The speed to drive at, as a percentage of full speed
   *                     (-1.0 to +1.0)
   * @param meters       The distance to drive, in meters.
   */
  public DriveForDistance(AbstractDrivebase drivebase, double percentSpeed, double meters) {
    this(drivebase, percentSpeed, Meters.of(meters));
  }

  /**
   * Creates a new DriveForDistance.
   *
   * Note that there's a bug in this constructor: it doesn't handle cases when the
   * speed says "move forward", but the distance says "move backward" or vice
   * versa.
   *
   * @param drivebase    The drivebase to use.
   * @param percentSpeed The speed to drive at, as a percentage of full speed
   *                     (-1.0 to +1.0)
   * @param distance     The distance to drive.
   */
  public DriveForDistance(AbstractDrivebase drivebase, double percentSpeed, Distance distance) {
    m_drivebase = drivebase;
    m_percentSpeed = (distance.baseUnitMagnitude() > 0 ? 1 : -1) * Math.abs(percentSpeed);
    m_distance = distance;
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stopAtPosition = m_drivebase.getLeftPosition().plus(m_distance);
    m_drivebase.setSpeeds(m_percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.setSpeeds(m_percentSpeed);
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
      // Desired distance was positive, so we're moving forward
      return m_drivebase.getLeftPosition().gte(m_stopAtPosition);
    } else {
      // Desired distance was negative, so we're moving backward
      return m_drivebase.getLeftPosition().lte(m_stopAtPosition);
    }
  }
}
