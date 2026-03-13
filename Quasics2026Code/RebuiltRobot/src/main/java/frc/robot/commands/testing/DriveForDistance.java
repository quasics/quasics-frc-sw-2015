// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.IDrivebase;

public class DriveForDistance extends Command {
  private static final boolean LOG_VELOCITY = true;

  private final IDrivebase m_drivebase;
  private final double m_percent;
  private final Distance m_distance;
  private final Timer m_timer = new Timer();
  private Distance m_targetDistance;
  private Distance m_lastReportedDistance;
  private double m_lastReportedTime;

  /**
   * Constructor.
   * 
   * @param drivebase drivebase being controlled
   * @param percent   percent speed to move at ([-1.0] to [+1.0]); this will be
   *                  normalized to match the direction associated with distance
   * @param distance  distance to be travelled (as reported by the drive base's
   *                  left-side encoder); negative distances indicate moving
   *                  backward
   */
  public DriveForDistance(IDrivebase drivebase, double percent, Distance distance) {
    m_drivebase = drivebase;
    m_distance = distance;

    // Normalize the percentage, making sure that its sign matches that of the
    // distance.
    m_percent = Math.abs(percent) * Math.signum(distance.magnitude());

    addRequirements((Subsystem) drivebase);
  }

  @Override
  public void initialize() {
    final Distance currentPosition = m_drivebase.getLeftDistance();
    m_lastReportedDistance = currentPosition;
    m_targetDistance = currentPosition.plus(m_distance);
    m_lastReportedTime = m_timer.get();
    m_drivebase.setPercent(m_percent, m_percent);
    m_timer.restart();

    // Enable braking mode
    if (!m_drivebase.setBreakingMode(true)) {
      System.err.println("*** Warning: couldn't enable braking mode for drivebase.");
      System.err.println("*** This may impact test data.");
    }

    // Debugging output (reporting starting conditions).
    System.out.println(
        "Starting driving at " + m_percent + " power, from " + m_lastReportedDistance + " to " + m_targetDistance);
  }

  static final double GEARING_RATIO = Constants.DRIVEBASE_GEAR_RATIO;

  @Override
  public void execute() {
    // Get current conditions.
    final double now = m_timer.get();
    final Distance currentDistance = m_drivebase.getLeftDistance();

    // Optional logging of current "step" in conditions.
    if (LOG_VELOCITY) {
      final Time sampleTime = Seconds.of(now - m_lastReportedTime);
      final Distance movementSinceLastSample = currentDistance.minus(m_lastReportedDistance);
      final LinearVelocity sampleVelocity = movementSinceLastSample.div(sampleTime);
      System.out.format(
          "Reported left distance: %.4f m (delta: %.4f m, rawMotor: %.4f rotations, withGearing: %.4f rotations), velocity: %.4f m/s (sampled: %.2f)\n",
          currentDistance.in(Meters),
          movementSinceLastSample.in(Meters),
          m_drivebase.getLeftRawDistance(),
          m_drivebase.getLeftRawDistance() / GEARING_RATIO,
          m_drivebase.getLeftVelocity().in(MetersPerSecond),
          sampleVelocity.in(MetersPerSecond));
    }

    // Retain values for next report.
    m_lastReportedDistance = currentDistance;
    m_lastReportedTime = now;

    // Don't forget to "feed" the differential drivebase.
    m_drivebase.setPercent(m_percent, m_percent);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    boolean result;
    if (m_distance.baseUnitMagnitude() >= 0) {
      // Desired distance was positive, so we're moving forward
      result = m_drivebase.getLeftDistance().gte(m_targetDistance);
      System.out.println("Forward - isFinished --> " + result);
    } else {
      // Desired distance was negative, so we're moving backward
      result = m_drivebase.getLeftDistance().lte(m_targetDistance);
      System.out.println("Backward - isFinished --> " + result);
    }
    return result;
  }
}
