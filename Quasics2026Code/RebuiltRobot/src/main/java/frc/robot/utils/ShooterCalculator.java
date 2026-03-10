// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
// FINDME(Daniel, Riley): Suggestion - Make the keys/values being passed into
// this class *Unit-based* values, and then convert them internally to inches
// (or meters, etc.) and RPM (or RPS, etc.). (I've added some examples of this
// to your code.)
//
// If you don't do this, then at least document clearly what your units should
// be. (I'm guessing inches, but it's good to make this explicit.)
public class ShooterCalculator {
  private final LinearInterpolator m_speedInterpolator = new LinearInterpolator();

  public ShooterCalculator() {
  }

  public void addDataPoint(double distanceInInches, double shooterSpeedRpm) {
    m_speedInterpolator.addDataPoint(distanceInInches, shooterSpeedRpm);
  }

  public double approximateRPM(double distanceInInches) {
    return m_speedInterpolator.getTargetApproximationForKey(distanceInInches);
  }

  public void addDataPoint(Distance distanceFromHub, AngularVelocity shooterSpeed) {
    m_speedInterpolator.addDataPoint(distanceFromHub.in(Inches), shooterSpeed.in(RPM));
  }

  public AngularVelocity approximateSpeed(Distance distanceFromHub) {
    return RPM.of(m_speedInterpolator.getTargetApproximationForKey(distanceFromHub.in(Inches)));
  }

  public static Translation2d getHubCenterLocation() {
    Distance xValue = Constants.RebuiltFieldData.ALLIANCE_WALL_TO_HUB_CENTER;

    Distance yValue = Constants.RebuiltFieldData.FIELD_WIDTH_CENTER;

    return new Translation2d(xValue, yValue);
  }

  public AngularVelocity getSpeedToHitHubCenter(Pose2d robotPose) {
    final Distance distanceToHub = TargetPositioningUtils.getDistanceToHubCenter(robotPose);
    SmartDashboard.putNumber("shooter key", distanceToHub.in(Inches));
    return approximateSpeed(distanceToHub);
  }
}