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
 * Simple class for handling linear interpolation.
 * 
 * You can think of this as implementing some sort of an algebraic function
 * ("f(x) -> y"), where one of these objects can be given a (relatively)
 * arbitrary "x" (key), and estimate the corresponding "y".
 * 
 * One obvious application is to try to estimate shooter speeds required to hit
 * a target from different distances.
 * 
 * Note that this is *linear* interpolation, and thus it's assumed that only
 * *one* value is being varied. (For the shooter speed example, this would be
 * the distance, and we'd assume that other things like the angle of a hood on
 * the shooter, etc., were not being changed.)
 */
public class LinearInterpolator {
  private final TreeMap<Double, Double> m_dataLookupTable = new TreeMap<>();

  /**
   * Constructor.
   * 
   * Note that data points will need to be added to the class before we can
   * generate estimates.
   * 
   * @see #addDataPoint(double, double)
   */
  public LinearInterpolator() {
  }

  /**
   * Adds a data point for use in estimation. (If you think of this class as
   * providing a function that maps "x" keys to "y" values, this adds a point on
   * the curve.)
   * 
   * @param key   a new known input ("x") value for the function
   * @param value a new known output ("y") value for the function
   */
  public void addDataPoint(double key, double value) {
    m_dataLookupTable.put(key, value);
  }

  public String toString() {
    return m_dataLookupTable.toString();
  }

  /**
   * Given a key (e.g., distance), generates an estimate of the corresponding
   * value (e.g., shooter speed) associated with it.
   * 
   * @param key value for which a corresponding estimate should be generated
   * @return an estimated value for the key
   * @throws IllegalStateException when we don't have any data points to use for
   *                               estimation
   */
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

  /**
   * Sample (though useful) class, demonstrating how the LinearInterpolator class
   * can be used in a "unitized" (unit-safe) form.
   */
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
