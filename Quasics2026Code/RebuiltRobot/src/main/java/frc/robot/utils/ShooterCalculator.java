// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
/**
 * 
 * 
 * FINDME(Daniel, Riley): Suggestion - Make the keys/values being passed into
 * this class *Unit-based* values, and then convert them internally to inches
 * (or meters, etc.) and RPM (or RPS, etc.).
 * 
 * I've added some examples of this to your code, as well as a new "inner class"
 * (LinearInterpolator.DistanceToShooterSpeedInterpolator) that also
 * demonstrates this.
 *
 * If you don't do this, then you should at least clealy document what your
 * units should be for both the distances and speeds. I'm guessing inches (from
 * the sample values) and RPMs (from the original comments), but it's important
 * to make this type of thing *explicit*.
 * 
 * @see frc.robot.utils.LinearInterpolator
 * @see frc.robot.utils.LinearInterpolator.DistanceToShooterSpeedInterpolator
 */
public class ShooterCalculator {
  /** Linear interpolator (holding inches->RPM data), used for estimation. */
  // FINDME(Daniel, Rylie): I'd suggest replacing this with a
  // LinearInterpolator.DistanceToShooterSpeedInterpolator object, which is
  // unit-safe. (See comments above.)
  private final LinearInterpolator m_speedInterpolator = new LinearInterpolator();

  /** Constructor. */
  public ShooterCalculator() {
  }

  /**
   * Adds a new data point for estimation purposes.
   * 
   * @param distanceInInches distance to the target (in inches)
   * @param shooterSpeedRpm  required shooter speed (in RPM) at the specified
   *                         distance
   */
  public void addDataPoint(double distanceInInches, double shooterSpeedRpm) {
    m_speedInterpolator.addDataPoint(distanceInInches, shooterSpeedRpm);
  }

  /**
   * Generates an estimate (in RPM) of the required shooting speed for the
   * specified distance.
   * 
   * @param distanceInInches distance to the target (in inches)
   * @return required shooter speed (in RPM) at the specified distance
   */
  public double approximateRPM(double distanceInInches) {
    return m_speedInterpolator.getTargetApproximationForKey(distanceInInches);
  }

  /**
   * Adds a new data point for estimation purposes.
   * 
   * @param distanceFromHub distance to the target
   * @param shooterSpeed    required shooter speed at the specified distance
   */
  public void addDataPoint(Distance distanceFromHub, AngularVelocity shooterSpeed) {
    m_speedInterpolator.addDataPoint(distanceFromHub.in(Inches), shooterSpeed.in(RPM));
  }

  /**
   * Generates an estimate of the required shooting speed at the specified
   * distance.
   * 
   * @param distanceFromHub distance to the target
   * @return required shooter speed at the specified distance
   */
  public AngularVelocity approximateSpeed(Distance distanceFromHub) {
    return RPM.of(m_speedInterpolator.getTargetApproximationForKey(distanceFromHub.in(Inches)));
  }

  public AngularVelocity getSpeedToHitHubCenter(Pose2d robotPose) {
    final Distance distanceToHub = TargetPositioningUtils.getDistanceToHubCenter(robotPose);
    SmartDashboard.putNumber("shooter key", distanceToHub.in(Inches));
    return approximateSpeed(distanceToHub);
  }
}