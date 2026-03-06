// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.TreeMap;

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
}
