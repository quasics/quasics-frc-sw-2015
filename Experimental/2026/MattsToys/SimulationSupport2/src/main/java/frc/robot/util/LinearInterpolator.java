// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import java.util.TreeMap;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Simple class for handling linear interpolation (e.g., calculating estimated
 * shooter velocity, given a range of "good values" at known distances).
 */
public class LinearInterpolator {
  private final TreeMap<Double, Double> m_dataLookupTable = new TreeMap<>();

  public LinearInterpolator() {
  }

  public void addDataPoint(double key, double value) {
    m_dataLookupTable.put(key, value);
  }

  public String toString() {
    return m_dataLookupTable.toString();
  }

  public double getTargetApproximationForKey(double key) {
    if (m_dataLookupTable.isEmpty()) {
      throw new IllegalStateException("No data to use in performing interpolation");
    }
    // Find the floor and ceiling entries in the lookup table
    var floorPair = m_dataLookupTable.floorEntry(key);
    var ceilPair = m_dataLookupTable.ceilingEntry(key);

    // Handle when we're outside the table bounds
    if (floorPair == null)
      return ceilPair.getValue();
    if (ceilPair == null)
      return floorPair.getValue();

    // Handle an exact match in the table.
    if (floorPair.getKey() == key) {
      return floorPair.getValue();
    }

    // Simple linear interpolation
    final double floorKey = floorPair.getKey();
    final double floorValue = floorPair.getValue();
    final double ceilKey = ceilPair.getKey();
    final double ceilValue = ceilPair.getValue();
    double speed = floorValue + (ceilValue - floorValue) * (key - floorKey) / (ceilKey - floorKey);
    return speed;
  }

  static public final class DistanceToShooterSpeedInterpolator {
    /**
     * Linear interpolation object we'll use.
     * 
     * Note that this will internally map distances to inches and speeds to RPMs, in
     * order to ensure data consistency. (This also means that we can *accept*
     * distances in any unit that the client code has handy. and they'll be able to
     * safely convert results into whatever is *needed*.)
     */
    private LinearInterpolator m_interpolator = new LinearInterpolator();

    /**
     * Constructor.
     * 
     * Note that data points will need to be added to the class before we can
     * generate estimates.
     * 
     * @see #addDataPoint(Distance, AngularVelocity)
     */
    public DistanceToShooterSpeedInterpolator() {
    }

    /**
     * Adds a data point for use in estimating needed speeds.
     * 
     * @param distance distance to the target
     * @param speed    (known) speed required at that distance
     */
    public void addDataPoint(Distance distance, AngularVelocity speed) {
      m_interpolator.addDataPoint(distance.in(Inches), speed.in(RPM));
    }

    /**
     * Returns an estimate of the speed needed to hit the target from the specified
     * distance.
     * 
     * @param distance current distance from the target
     * @return estimated speed for the shooter, in order to hit the target
     * @throws IllegalStateException when we don't have any data points to use for
     *                               estimation
     */
    public AngularVelocity getEstimatedShooterSpeed(Distance distance) {
      return RPM.of(m_interpolator.getTargetApproximationForKey(distance.in(Inches)));
    }
  }
}
