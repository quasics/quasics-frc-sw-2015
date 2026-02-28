// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Implements a simple command to drive for a given distance, at a set speed.
 *
 * Suggestion from WPILib template: "You should consider using the more terse
 * Command factories API instead."
 *
 * @see
 *      https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
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
   * (This will assume that we want to go at 50% speed.)
   * 
   * @param drivebase The drivebase to use.
   * @param distance  The distance to drive.
   * 
   * @see #DriveForDistance(AbstractDrivebase, double, Distance)
   */
  public DriveForDistance(AbstractDrivebase drivebase, Distance distance) {
    this(drivebase, .5, distance);
  }

  /**
   * Creates a new DriveForDistance, using our *other* constructor to do the work.
   *
   * @param drivebase    The drivebase to use.
   * @param percentSpeed The speed to drive at, as a percentage of full speed
   *                     (-1.0 to +1.0)
   * @param meters       The distance to drive, in meters.
   * 
   * @see #DriveForDistance(AbstractDrivebase, double, Distance)
   */
  public DriveForDistance(AbstractDrivebase drivebase, double percentSpeed, double meters) {
    this(drivebase, percentSpeed, Meters.of(meters));
  }

  /**
   * Creates a new DriveForDistance.
   *
   * Note that there's potential for conflicting data, such as when the speed says
   * "move forward", but the distance says "move backward" or vice versa. We'll
   * handle this by ignoring the sign on the speed, and just let the distance
   * determine "forward" vs. "backward".
   *
   * @param drivebase    The drivebase to use.
   * @param percentSpeed The speed to drive at, as a percentage of full speed
   *                     (-1.0 to +1.0)
   * @param distance     The distance to drive.
   */
  public DriveForDistance(AbstractDrivebase drivebase, double percentSpeed, Distance distance) {
    m_drivebase = drivebase;
    m_distance = distance;

    // Extract the underlying (signed) number for the distance. (Also demonstrates
    // converting a "Distance" value to a specific kind of units.)
    final double rawDistance = distance.in(Meters);

    // Use the sign of the underlying number to make sure that the targeted speed
    // has an appropriate (matching) sign (positive/negative)
    m_percentSpeed = Math.signum(rawDistance) * Math.abs(percentSpeed);

    // Tell WPILib that when this command is running, it's using the drive base (so
    // no one else should be allowed to).
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out what the stopping point is.
    m_stopAtPosition = m_drivebase.getLeftDistance().plus(m_distance);

    // Start moving.
    m_drivebase.setSpeed(m_percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // (optional) Keep moving at that speed.
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
      // Desired distance was positive, so we're moving forward
      return m_drivebase.getLeftDistance().gte(m_stopAtPosition);
    } else {
      // Desired distance was negative, so we're moving backward
      return m_drivebase.getLeftDistance().lte(m_stopAtPosition);
    }
  }
}
