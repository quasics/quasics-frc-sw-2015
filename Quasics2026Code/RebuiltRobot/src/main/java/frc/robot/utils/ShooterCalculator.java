// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ShooterCalculator {
  private final TreeMap<Double, Double> m_speedCalculationTable = new TreeMap<>();

  public ShooterCalculator() {

  }

  public void addDataPoint(double key, double value) {
    m_speedCalculationTable.put(key, value);
  }

  public double approximateRPM(double key) {
    if (m_speedCalculationTable.isEmpty()) {
      throw new IllegalStateException("No data");
    }
    var floor = m_speedCalculationTable.floorEntry(key); // entry below key number
    var ceiling = m_speedCalculationTable.ceilingEntry(key);
    if (floor == null) {
      return ceiling.getValue();
    }
    if (ceiling == null) {
      return floor.getValue();
    }
    if (floor.getKey() == key) {
      return floor.getValue();
    }

    final double floorKey = floor.getKey();
    final double floorValue = floor.getValue();
    final double ceilingKey = ceiling.getKey();
    final double ceilingValue = ceiling.getValue();
    // (how far between floor and ceiling number * gap between value of floor/ceil)
    // to get how much speed needs to be added to floorval
    double speed = floorValue + (ceilingValue - floorValue) * (key - floorKey) / (ceilingKey - floorKey);
    return speed;
  }

  public static Translation2d getHubCenterLocation() {
    Distance xValue = Constants.RebuiltFieldData.ALLIANCE_WALL_TO_HUB_CENTER;

    Distance yValue = Constants.RebuiltFieldData.FIELD_WIDTH_CENTER;

    return new Translation2d(xValue, yValue);
  }

  // translation parameter
  public static Distance getDistanceToHubCenter(Translation2d robotPosition) {
    Translation2d hubCenter = getHubCenterLocation();

    return Meters.of(robotPosition.getDistance(hubCenter));
  }

  // pose parameter
  public static Distance getDistanceToHubCenter(Pose2d robotPose) {
    return getDistanceToHubCenter(robotPose.getTranslation());
  }

  public double getSpeedToHitHubCenter(Pose2d robotPose) {
    final double distance = getDistanceToHubCenter(robotPose).in(Meters) * 39.3701;

    System.out.println("dist " + distance + "POse " + robotPose);
    SmartDashboard.putNumber("shooter key", distance);
    return approximateRPM(distance);
  }
}